/**
 * @brief 轮足机器人 USB-CDC 通信节点（与 stm32/one/task 固件对齐）
 *
 * 唯一的实机通信出口，只做“搬运”，不做任何控制计算：
 *
 * 下行：按 send_rate 周期发送 RobotTarget_t（104 B）
 *   wheel[0..1] : wheel_torque（[τ_L, τ_R]）→ 力矩直通（wheel_torque_gain，实机必须 1.0）；
 *                 wheel_target（[ω_L, ω_R]）→ 只填 angle/omega 保留字段（固件已不读）
 *   joint[0..3] : 由 joint_target（sensor_msgs/JointState）的 4 个腿关节 + 参数 kp/kd 组成
 * 上行：解析 RobotState_t（208 B，固件 100 Hz 上报）
 *   wheel[0..1] : M2006 轮子角度/角速度
 *   joint[0..3] : RobStride 关节反馈
 *
 * 注意：固件按结构体裸内存收发，没有包头和长度字段，因此上位机必须按固定长度拆帧；
 *       USB bulk 可能出现粘包/拆包，这里用累积缓冲区重组。
 *
 * 频率：默认 200 Hz 下发（下位机控制环 250 Hz、状态上报 100 Hz）。
 * 三重兜底：
 *   ① 控制指令超时（command_timeout）→ 轮子力矩置 0（失力）、关节失力
 *   ② 状态帧超时（state_timeout）→ 立即改发安全帧，不再下发过期指令
 *   ③ 节点退出 → 再发一帧安全帧
 *
 * 话题：
 *   订阅 wheel_target    (Float64MultiArray, [左, 右])  —— 轮速目标 [rad/s]（实机不生效，仅填保留字段）
 *   订阅 wheel_torque    (Float64MultiArray, [左, 右])  —— 轮轴力矩 [N*m]（实机主通道）
 *   订阅 joint_target    (sensor_msgs/JointState)       —— 4 个腿关节目标（effort=力矩）
 *   （不发布 odom：里程计由 leg_control 用本节点上报的轮端数据换算）
 *   发布 joint_states    (sensor_msgs/JointState)       —— 2 轮 + 4 关节反馈
 *   发布 usb_drive/state (std_msgs/Float64MultiArray)   —— 原始反馈
 */

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "usb_drive/cdc_trans.hpp"
#include "usb_drive/data_pack.h"

// 与固件逐字节对齐校验统一放在 data_pack.h 里（那里是协议的唯一来源），
// 这里只需保证 target 帧两段偏移与固件 task.c 一致：
static_assert(sizeof(usb_drive::RobotTarget_t) == 104, "RobotTarget_t 布局与固件不一致");

namespace
{

/// 500 Hz 热路径（/wheel_target、/wheel_torque、/joint_target、/joint_states）
/// 统一 QoS：只保留最新一帧 + best_effort。
///
/// 可靠传输下，本节点只要有一次没及时取走消息（USB 同步写最长 5 ms/帧），
/// 上游 leg_control 的 publish() 就会被反压阻塞 —— 而 leg_control 的 500 Hz
/// 控制环一旦阻塞超过 0.1 s，三条反馈同时超时，机器人直接 0x23 停机。
/// 控制指令只需“最新值”，丢帧下一周期就补上，所以宁可丢帧也不能阻塞。
rclcpp::QoS hotPathQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
}

/// 帧合理性判断，用于失步后丢弃脏数据
///
/// 只校验数值健全性（float 有限、量程合理）。`feedback` 完全不参与判定：
/// 它是下位机的状态字而不是 0~8 的枚举，实测电机未使能时报 0x80(128)，
/// 若拿它当“非法帧”判据会误判并形成死锁——收不到状态帧 → 不发 joint_states →
/// leg_control 一直 INIT 故障 → 轮子/关节永远得不到使能指令。
/// `bad_joint` 回传首个越界的关节下标（-1 = 轮子或整体越界），便于日志定位。
bool plausibleState(const usb_drive::RobotState_t & state, int * bad_joint = nullptr)
{
  if (bad_joint != nullptr) {
    *bad_joint = -1;
  }
  for (const auto & wheel : state.wheel) {
    if (!std::isfinite(wheel.angle) || !std::isfinite(wheel.omega) ||
      std::abs(wheel.angle) > 1e4f || std::abs(wheel.omega) > 1e3f)
    {
      return false;
    }
  }
  for (size_t i = 0; i < 4; ++i) {
    const auto & joint = state.joint[i];
    if (!std::isfinite(joint.rad) || !std::isfinite(joint.omega) ||
      !std::isfinite(joint.torque) || std::abs(joint.rad) > 1e3f)
    {
      if (bad_joint != nullptr) {
        *bad_joint = static_cast<int>(i);
      }
      return false;
    }
  }
  return true;
}

/// `feedback` 是状态字（bit 标志），取值空间没有 0~8 的约定，
/// 因此这里不做“合法性”判断，只在其发生变化时打印一次，便于确认链路状态。
void logFeedbackChange(
  rclcpp::Logger logger, const usb_drive::RobotState_t & state,
  uint8_t last[4], bool & initialized)
{
  if (!initialized) {
    initialized = true;
    for (size_t i = 0; i < 4; ++i) {
      last[i] = state.joint[i].feedback;
    }
    RCLCPP_INFO(
      logger, "关节反馈状态字：%u/%u/%u/%u（仅在变化时打印，不作合法性判据）",
      static_cast<unsigned>(last[0]), static_cast<unsigned>(last[1]),
      static_cast<unsigned>(last[2]), static_cast<unsigned>(last[3]));
    return;
  }
  for (size_t i = 0; i < 4; ++i) {
    const uint8_t fb = state.joint[i].feedback;
    if (fb != last[i]) {
      RCLCPP_INFO(
        logger, "关节%zu 反馈状态字 %u → %u", i,
        static_cast<unsigned>(last[i]), static_cast<unsigned>(fb));
      last[i] = fb;
    }
  }
}

}  // namespace

class UsbDriveNode : public rclcpp::Node
{
public:
  UsbDriveNode()
  : Node("usb_drive_node")
  {
    // ---------------- USB-CDC 设备参数 ----------------
    vid_ = static_cast<uint16_t>(declare_parameter("vid", 0x0483));
    pid_ = static_cast<uint16_t>(declare_parameter("pid", 0x5740));
    interface_number_ = static_cast<int>(declare_parameter("interface", 1));
    ep_in_ = static_cast<uint8_t>(declare_parameter("ep_in", 0x81));
    ep_out_ = static_cast<uint8_t>(declare_parameter("ep_out", 0x01));

    // ---------------- 底盘参数（只保留方向与限幅，物理量由 leg_control 持有） ----------------
    left_invert_ = declare_parameter("left_invert", false);
    right_invert_ = declare_parameter("right_invert", false);
    max_wheel_omega_ = declare_parameter("max_wheel_omega", 20.0);

    // ---------------- 关节参数 ----------------
    // 顺序 = 帧里 joint[0..3] 的顺序 = [左前, 左后, 右前, 右后]，必须与 URDF/leg_calc 一致
    leg_joint_names_ = declare_parameter(
      "leg_joint_names",
      std::vector<std::string>{"lf_joint1", "lb_joint1", "rf_joint1", "rb_joint1"});
    wheel_joint_names_ = declare_parameter(
      "wheel_joint_names", std::vector<std::string>{"lefoot_joint", "rifoot_joint"});
    joint_kp_ = declare_parameter("joint_kp", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    joint_kd_ = declare_parameter("joint_kd", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    if (leg_joint_names_.size() != 4 || wheel_joint_names_.size() != 2) {
      throw std::invalid_argument("leg_joint_names 需 4 个、wheel_joint_names 需 2 个元素");
    }
    if (joint_kp_.size() != 4 || joint_kd_.size() != 4) {
      throw std::invalid_argument("joint_kp 与 joint_kd 各需 4 个元素");
    }

    // ---------------- 通信参数 ----------------
    // 200 Hz 下发：与“500 Hz 控制环 / 50 Hz 上层命令”解耦，只保证命令搬运带宽
    command_timeout_ = declare_parameter("command_timeout", 0.3);
    state_timeout_ = declare_parameter("state_timeout", 0.5);
    send_rate_ = declare_parameter("send_rate", 200.0);

    max_wheel_torque_ = declare_parameter("max_wheel_torque", 1.0);
    // /wheel_torque → 帧里 wheel[i].torque 的缩放。
    // 固件 wheel_control_task 是**纯力矩直通**（无位置/速度环，只读 wheel[].torque），
    // 所以这是实机唯一的轮子执行通道，默认必须 1.0；置 0 等于轮子完全不出力。
    // 同时要把 leg_control 的 wheel_torque_enable 打开（否则 /wheel_torque 恒为 0）。
    wheel_torque_gain_ = declare_parameter("wheel_torque_gain", 1.0);

    const std::string wheel_target_topic =
      declare_parameter("wheel_target_topic", std::string("wheel_target"));
    const std::string wheel_torque_topic =
      declare_parameter("wheel_torque_topic", std::string("wheel_torque"));
    const std::string joint_target_topic =
      declare_parameter("joint_target_topic", std::string("joint_target"));
    const std::string joint_states_topic =
      declare_parameter("joint_states_topic", std::string("joint_states"));

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, hotPathQoS());
    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("usb_drive/state", 10);

    wheel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      wheel_target_topic, hotPathQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {onWheelTarget(*msg);});
    wheel_torque_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      wheel_torque_topic, hotPathQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {onWheelTorque(*msg);});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_target_topic, hotPathQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJointTarget(*msg);});

    // ---------------- 启动 CDC ----------------
    assembler_ = std::make_unique<usb_drive::FrameAssembler>(
      sizeof(usb_drive::RobotState_t),
      [this](const uint8_t * frame, size_t size) {onFrame(frame, size);});

    cdc_ = std::make_unique<usb_drive::CdcTrans>();
    cdc_->registerRecvCallback(
      [this](const uint8_t * data, int size) {
        if (size > 0) {
          assembler_->feed(data, static_cast<size_t>(size));
        }
      });
    if (!cdc_->open(vid_, pid_, interface_number_, ep_in_, ep_out_)) {
      RCLCPP_WARN(
        get_logger(), "USB-CDC 打开失败，后台线程将每 500 ms 重试（检查设备与 udev 权限）");
    }

    exit_thread_ = false;
    event_thread_ = std::thread(
      [this]() {
        while (!exit_thread_) {
          cdc_->processOnce();
        }
      });

    // 下发独立线程：sendTarget() 里是同步 libusb_bulk_transfer（单帧最长 5 ms），
    // 放在 ROS 定时器回调里会占住 executor —— 订阅队列随即堆满，把所有上游
    // 发布者（leg_control 的 500 Hz 控制环）一起拖死。挪出 executor 后，
    // ROS 侧只做“收消息、存最新值”，USB 侧自己按周期取最新值发送。
    send_thread_ = std::thread([this]() {sendLoop();});

    RCLCPP_INFO(
      get_logger(),
      "USB-CDC 通信节点启动: %04x:%04x 接口%d, 收发 %zu/%zu B, 发送 %.0f Hz（独立线程）",
      vid_, pid_, interface_number_, sizeof(usb_drive::RobotTarget_t),
      sizeof(usb_drive::RobotState_t), send_rate_);
    RCLCPP_INFO(
      get_logger(),
      "轮子执行通道: 固件力矩直通（主通道 /wheel_torque）；力矩 gain=%.2f（0 = 轮子不出力）",
      wheel_torque_gain_);
  }

  ~UsbDriveNode() override
  {
    exit_thread_ = true;
    if (send_thread_.joinable()) {
      send_thread_.join();
    }
    if (event_thread_.joinable()) {
      event_thread_.join();
    }
    if (cdc_) {
      // 退出时下发“保持轮角 + 阻尼制动 + 关节失力”的安全帧
      usb_drive::RobotTarget_t safe{};
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        fillSafeFrame(&safe);
      }
      cdc_->sendStruct(safe);
      cdc_->close();
    }
  }

private:
  // ---------------- 下行：左右轮目标转速 ----------------
  void onWheelTarget(const std_msgs::msg::Float64MultiArray & msg)
  {
    if (msg.data.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "wheel_target 需要 2 个数 [左, 右]，忽略本帧");
      return;
    }
    const rclcpp::Time stamp = now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    wheel_target_speed_[0] = msg.data[0];
    wheel_target_speed_[1] = msg.data[1];
    last_wheel_target_time_ = stamp;
    wheel_target_received_ = true;
  }

  // ---------------- 下行：左右轮力矩（直通成 M2006 电流） ----------------
  void onWheelTorque(const std_msgs::msg::Float64MultiArray & msg)
  {
    if (msg.data.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "wheel_torque 需要 2 个数 [左, 右]，忽略本帧");
      return;
    }
    const rclcpp::Time stamp = now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < 2; ++i) {
      const double v = msg.data[i];
      wheel_torque_[i] = std::isfinite(v) ?
        std::clamp(v, -max_wheel_torque_, max_wheel_torque_) : 0.0;
    }
    last_wheel_torque_time_ = stamp;
    wheel_torque_received_ = true;
  }

  // ---------------- 下行：4 个腿关节目标 ----------------
  void onJointTarget(const sensor_msgs::msg::JointState & msg)
  {
    usb_drive::JointTarget_t next[4]{};
    bool all_found = true;
    for (size_t i = 0; i < leg_joint_names_.size(); ++i) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), leg_joint_names_[i]);
      if (it == msg.name.end()) {
        all_found = false;
        break;
      }
      const size_t index = static_cast<size_t>(std::distance(msg.name.begin(), it));
      next[i].angle =
        index < msg.position.size() ? static_cast<float>(msg.position[index]) : 0.0f;
      next[i].omega =
        index < msg.velocity.size() ? static_cast<float>(msg.velocity[index]) : 0.0f;
      next[i].torque =
        index < msg.effort.size() ? static_cast<float>(msg.effort[index]) : 0.0f;
    }
    if (!all_found) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "joint_target 缺少关节名 %s 等，忽略本帧",
        leg_joint_names_[0].c_str());
      return;
    }
    const rclcpp::Time stamp = now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < 4; ++i) {
      joint_target_[i] = next[i];
    }
    last_joint_target_time_ = stamp;
    joint_target_received_ = true;
  }

  // ---------------- 上行：一帧完整状态 ----------------
  void onFrame(const uint8_t * frame, size_t size)
  {
    if (size != sizeof(usb_drive::RobotState_t)) {
      return;
    }
    usb_drive::RobotState_t state{};
    std::memcpy(&state, frame, size);  // 缓冲区不保证对齐
    int bad_joint = -1;
    if (!plausibleState(state, &bad_joint)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "收到非法状态帧（关节%d 数值越界），丢弃本帧，已连续 %zu 帧",
        bad_joint, bad_frame_count_);
      // 协议无包头，reset() 只能把切分点挪到下一个任意字节，并不能真正重新对齐；
      // 反而会丢掉缓冲区里已完整的后续帧。所以只丢本帧，连续大量失败才清缓冲碰运气。
      if (++bad_frame_count_ >= 50) {
        bad_frame_count_ = 0;
        assembler_->reset();
      }
      return;
    }
    bad_frame_count_ = 0;

    logFeedbackChange(
      get_logger(), state, feedback_last_, feedback_logged_);
    handleState(state);
  }

  void handleState(const usb_drive::RobotState_t & state)
  {
    const rclcpp::Time stamp = now();

    const double dir_l = left_invert_ ? -1.0 : 1.0;
    const double dir_r = right_invert_ ? -1.0 : 1.0;

    const double angle_l = dir_l * state.wheel[0].angle;
    const double angle_r = dir_r * state.wheel[1].angle;
    const double omega_l = dir_l * state.wheel[0].omega;
    const double omega_r = dir_r * state.wheel[1].omega;

    {
      // 下发线程要读这些量组装帧；临界区只有几次赋值
      std::lock_guard<std::mutex> lock(data_mutex_);
      state_received_ = true;
      last_state_time_ = stamp;
      measured_wheel_angle_[0] = angle_l;
      measured_wheel_angle_[1] = angle_r;
      // 实测轮速（世界系，已含 dir）：安全帧的阻尼制动要用它
      measured_wheel_omega_[0] = omega_l;
      measured_wheel_omega_[1] = omega_r;
    }

    // 关节状态：2 轮 + 4 关节（里程计由 leg_control 用这些轮端数据换算）
    // 发布放在锁外：publish 若短暂阻塞，不能把下发线程一起拖住
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name = wheel_joint_names_;
    joints.name.insert(joints.name.end(), leg_joint_names_.begin(), leg_joint_names_.end());
    joints.position = {angle_l, angle_r};
    joints.velocity = {omega_l, omega_r};
    // 固件 Wheel_t 只回传角度与角速度，没有轮子力矩反馈
    joints.effort = {0.0, 0.0};
    for (size_t i = 0; i < 4; ++i) {
      joints.position.push_back(state.joint[i].rad);
      joints.velocity.push_back(state.joint[i].omega);
      joints.effort.push_back(state.joint[i].torque);
    }
    joint_pub_->publish(joints);

    // 原始反馈，便于调试
    std_msgs::msg::Float64MultiArray raw;
    raw.data = {angle_l, angle_r, omega_l, omega_r};
    for (size_t i = 0; i < 4; ++i) {
      raw.data.push_back(state.joint[i].rad);
      raw.data.push_back(state.joint[i].omega);
    }
    state_pub_->publish(raw);
  }

  // ---------------- 周期下发（独立线程） ----------------
  /// 固定周期调度：时间基准用 steady_clock 累加，避免 ROS 时钟跳变；
  /// 落后太多（USB 卡顿时）就重新对齐，不做补偿式连发。
  void sendLoop()
  {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / std::max(1.0, send_rate_)));
    auto next = std::chrono::steady_clock::now() + period;
    while (!exit_thread_) {
      std::this_thread::sleep_until(next);
      next += period;
      const auto now_tp = std::chrono::steady_clock::now();
      if (now_tp - next > period * 10) {
        next = now_tp + period;
      }
      sendTarget();
    }
  }

  void sendTarget()
  {
    const rclcpp::Time stamp = now();
    usb_drive::RobotTarget_t target{};

    {
      // 组帧只读“最新命令快照”，临界区里不做任何 IO / 日志级别的重活
      std::lock_guard<std::mutex> lock(data_mutex_);

      // 状态帧超时 = 链路可疑：不再继续下发过期指令，改发安全帧
      const bool link_ok =
        state_received_ && (stamp - last_state_time_).seconds() <= state_timeout_;
      if (!link_ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "下位机状态超过 %.2f s 未更新，下发安全帧（轮子保持 + 关节失力）", state_timeout_);
        fillSafeFrame(&target);
        // 恢复后重新从实测角度起积分，避免残留的目标角跳变
        wheel_angle_init_ = false;
      } else {
        // 用实际发送间隔积分目标角，避免定时抖动导致角度指令漂移
        const double dt = std::clamp((stamp - last_send_time_).seconds(), 1e-4, 0.1);
        last_send_time_ = stamp;

        // 轮子主指令来源：/wheel_target（[ω_L, ω_R]），超时即位置保持
        const bool wheel_fresh =
          wheel_target_received_ &&
          (stamp - last_wheel_target_time_).seconds() <= command_timeout_;
        if (!wheel_fresh && wheel_angle_init_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "轮速指令中断超过 %.3f s，停止积分保留字段 angle/omega（轮子力矩不受影响）",
            command_timeout_);
          wheel_angle_init_ = false;
        }

        // 主通道：/wheel_torque（[τ_L, τ_R]，N·m）→ 固件乘 WHEEL_COUNTS_PER_NM 变电流；超时即归零
        const bool torque_fresh =
          wheel_torque_received_ &&
          (stamp - last_wheel_torque_time_).seconds() <= command_timeout_;

        for (size_t i = 0; i < 2; ++i) {
          const bool invert = (i == 0) ? left_invert_ : right_invert_;
          // 上行读到的角度/角速度已乘过 dir，写目标角时必须反着变换回固件方向
          const double dir = invert ? -1.0 : 1.0;
          double angle_cmd = measured_wheel_angle_[i];
          double omega_cmd = 0.0;
          double torque_cmd = 0.0;

          // angle/omega 是协议保留字段（固件已不读），仍按 ω 目标积分填好，
          // 便于抓帧对照；仿真桥走的是另一条链路，不依赖这里。
          if (wheel_fresh) {
            omega_cmd = std::clamp(wheel_target_speed_[i], -max_wheel_omega_, max_wheel_omega_);
            if (!wheel_angle_init_ || omega_cmd == 0.0) {
              // 速度目标为 0（未使能 / FAULT / 停车）时把角度目标重新锚到实测角，
              // 避免保留字段长期累积出无意义的几圈偏差
              wheel_angle_target_[i] = measured_wheel_angle_[i];
              wheel_angle_init_ = true;
            }
            wheel_angle_target_[i] += omega_cmd * dt;
            angle_cmd = wheel_angle_target_[i];
          }
          if (torque_fresh && wheel_torque_gain_ != 0.0) {
            // 默认 gain=1.0：这是实机唯一的轮子执行通道（固件纯力矩直通）
            torque_cmd = std::clamp(
              wheel_torque_gain_ * wheel_torque_[i], -max_wheel_torque_, max_wheel_torque_);
          }

          target.wheel[i].angle = static_cast<float>(dir * angle_cmd);
          target.wheel[i].omega = static_cast<float>(dir * omega_cmd);
          target.wheel[i].torque = static_cast<float>(dir * torque_cmd);
        }

        const bool joint_fresh =
          joint_target_received_ &&
          (stamp - last_joint_target_time_).seconds() <= command_timeout_;
        for (size_t i = 0; i < 4; ++i) {
          if (joint_fresh) {
            target.joint[i] = joint_target_[i];
            target.joint[i].kp = static_cast<float>(joint_kp_[i]);
            target.joint[i].kd = static_cast<float>(joint_kd_[i]);
          } else {
            // kp/kd/torque 全 0：电机不受力（自由状态）
            target.joint[i].torque = 0.0f;
            target.joint[i].angle = 0.0f;
            target.joint[i].omega = 0.0f;
            target.joint[i].kp = 0.0f;
            target.joint[i].kd = 0.0f;
          }
        }
      }
    }

    // 同步 USB 写在锁外：单帧最长 5 ms，期间不能让订阅回调 / 收帧回调空等
    cdc_->sendStruct(target);
  }

  /// 组装安全帧：轮子力矩置 0（失力），关节全部失力，保留字段填实测角/零速
  ///
  /// 固件是纯力矩直通、没有位置环：torque = 0 就等于轮子自由（坡上会溜车），
  /// 想制动得填 τ = −k·ω（见 leg_control 的 wheel_brake_damp）。
  /// 调用方必须已持有 data_mutex_
  void fillSafeFrame(usb_drive::RobotTarget_t * safe) const
  {
    for (size_t i = 0; i < 2; ++i) {
      const bool invert = (i == 0) ? left_invert_ : right_invert_;
      const double dir = invert ? -1.0 : 1.0;
      safe->wheel[i].angle = static_cast<float>(dir * measured_wheel_angle_[i]);
      safe->wheel[i].omega = 0.0f;
      safe->wheel[i].torque = 0.0f;
    }
  }

  // CDC 参数
  uint16_t vid_{0x0483};
  uint16_t pid_{0x5740};
  int interface_number_{1};
  uint8_t ep_in_{0x81};
  uint8_t ep_out_{0x01};

  // 底盘参数
  bool left_invert_{false};
  bool right_invert_{false};
  double max_wheel_omega_{20.0};

  // 关节参数
  std::vector<std::string> leg_joint_names_;
  std::vector<std::string> wheel_joint_names_;
  std::vector<double> joint_kp_;
  std::vector<double> joint_kd_;
  usb_drive::JointTarget_t joint_target_[4]{};

  // 通信参数
  double command_timeout_{0.3};
  double state_timeout_{0.5};
  double send_rate_{200.0};

  /// 保护下面所有“跨线程共享的实时状态”：
  ///   写：ROS 订阅回调（命令）、USB 事件线程（反馈）
  ///   读：下发线程（组帧）
  /// 参数类（名字、kp/kd、限幅）只在构造时写，之后只读，无需加锁。
  std::mutex data_mutex_;

  // 控制指令缓存
  rclcpp::Time last_joint_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_send_time_{0, 0, RCL_ROS_TIME};
  bool joint_target_received_{false};

  // 链路健康
  bool state_received_{false};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};

  // 差速底盘轮速分量 [rad/s]
  double wheel_target_speed_[2]{0.0, 0.0};
  bool wheel_target_received_{false};
  rclcpp::Time last_wheel_target_time_{0, 0, RCL_ROS_TIME};

  // 力矩分量 [N*m]（固件纯力矩直通，这是实机唯一的轮子执行通道；gain=0 时不参与）
  double wheel_torque_[2]{0.0, 0.0};
  double wheel_torque_gain_{1.0};
  double max_wheel_torque_{1.0};
  bool wheel_torque_received_{false};
  rclcpp::Time last_wheel_torque_time_{0, 0, RCL_ROS_TIME};

  // 轮子目标角度积分
  double wheel_angle_target_[2]{0.0, 0.0};
  bool wheel_angle_init_{false};
  double measured_wheel_angle_[2]{0.0, 0.0};
  double measured_wheel_omega_[2]{0.0, 0.0};

  // 接收帧重组（仅 USB 事件线程访问）
  std::unique_ptr<usb_drive::FrameAssembler> assembler_;
  /// 连续被判定为非法的状态帧数，用于判断是否需要清缓冲重新碰运气
  size_t bad_frame_count_{0};
  /// 上一次的关节反馈状态字（只用于变化时打印，见 logFeedbackChange）
  uint8_t feedback_last_[4]{0, 0, 0, 0};
  bool feedback_logged_{false};

  std::unique_ptr<usb_drive::CdcTrans> cdc_;
  std::thread event_thread_;
  /// 下发线程（不占 ROS executor）
  std::thread send_thread_;
  std::atomic_bool exit_thread_{false};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_torque_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UsbDriveNode>());
  rclcpp::shutdown();
  return 0;
}
