/**
 * @brief 轮足机器人 USB-CDC 通信节点（与 stm32/one/task 固件对齐）
 *
 * 下行：按 send_rate 周期发送 RobotTarget_t（96 B）
 *   wheel[0..1] : 由 leg_calc/control 的 u 折算成轮子目标（速度/位置模式可选）
 *   joint[0..3] : 由 joint_target（sensor_msgs/JointState）的 4 个腿关节 + 参数 kp/kd 组成
 * 上行：解析 RobotState_t（208 B，固件 100 Hz 上报）
 *   wheel[0..1] : M2006 轮子角度/角速度
 *   joint[0..3] : RobStride 关节反馈
 *
 * 注意：固件按结构体裸内存收发，没有包头和长度字段，因此上位机必须按固定长度拆帧；
 *       USB bulk 可能出现粘包/拆包，这里用累积缓冲区重组。
 *
 * 话题：
 *   订阅 leg_calc/control (Float64MultiArray, [u])      —— LQR 输出（轮子平衡分量）
 *   订阅 wheel_target    (Float64MultiArray, [左, 右])  —— 差速底盘的轮速分量 [rad/s]
 *   订阅 joint_target    (sensor_msgs/JointState)       —— 4 个腿关节目标
 *   发布 odom            (nav_msgs/Odometry)            —— x, x_dot（供 LQR 使用）
 *   发布 joint_states    (sensor_msgs/JointState)       —— 2 轮 + 4 关节反馈
 *   发布 usb_drive/state (std_msgs/Float64MultiArray)   —— 原始反馈
 *
 * 轮子目标 = 平衡分量（u×gain，左右同向）+ 行驶分量（wheel_target，左右差速），
 * 任一路径超时即按 0 处理；两路都超时则轮子位置保持、关节失力。
 */

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "usb_drive/cdc_trans.hpp"
#include "usb_drive/data_pack.h"

// 与固件逐字节对齐校验（若固件结构体改动，这里会直接编译失败提醒）
static_assert(sizeof(usb_drive::Wheel_t) == 8, "Wheel_t 布局与固件不一致");
static_assert(sizeof(usb_drive::JointTarget_t) == 20, "JointTarget_t 布局与固件不一致");
static_assert(sizeof(usb_drive::JointState_t) == 48, "JointState_t 布局与固件不一致");
static_assert(sizeof(usb_drive::RobotTarget_t) == 96, "RobotTarget_t 布局与固件不一致");
static_assert(sizeof(usb_drive::RobotState_t) == 208, "RobotState_t 布局与固件不一致");
static_assert(offsetof(usb_drive::JointState_t, error) == 32, "JointState_t.error 偏移异常");
static_assert(offsetof(usb_drive::JointState_t, feedback) == 40, "JointState_t.feedback 偏移异常");

namespace
{

enum class WheelCmdMode
{
  kVelocity,  ///< u 折算成目标轮速，角度由上位机积分得到
  kPosition,  ///< u 直接作为目标轮角
};

/// 帧合理性判断，用于失步后丢弃脏数据
bool plausibleState(const usb_drive::RobotState_t & state)
{
  for (const auto & wheel : state.wheel) {
    if (!std::isfinite(wheel.angle) || !std::isfinite(wheel.omega) ||
      std::abs(wheel.angle) > 1e4f || std::abs(wheel.omega) > 1e3f)
    {
      return false;
    }
  }
  for (const auto & joint : state.joint) {
    if (!std::isfinite(joint.rad) || !std::isfinite(joint.omega) ||
      !std::isfinite(joint.torque) || std::abs(joint.rad) > 1e3f)
    {
      return false;
    }
    if (joint.feedback > 8) {
      return false;
    }
  }
  return true;
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

    // ---------------- 底盘参数 ----------------
    wheel_radius_ = declare_parameter("wheel_radius", 0.065);
    wheel_separation_ = declare_parameter("wheel_separation", 0.20);
    left_wheel_gain_ = declare_parameter("left_wheel_gain", 1.0);
    right_wheel_gain_ = declare_parameter("right_wheel_gain", 1.0);
    left_invert_ = declare_parameter("left_invert", false);
    right_invert_ = declare_parameter("right_invert", false);
    max_wheel_omega_ = declare_parameter("max_wheel_omega", 20.0);
    max_command_ = declare_parameter("max_command", 5.0);

    const std::string wheel_mode = declare_parameter("wheel_cmd_mode", std::string("velocity"));
    if (wheel_mode == "position") {
      wheel_cmd_mode_ = WheelCmdMode::kPosition;
    } else {
      wheel_cmd_mode_ = WheelCmdMode::kVelocity;
    }

    // ---------------- 关节参数 ----------------
    leg_joint_names_ = declare_parameter(
      "leg_joint_names",
      std::vector<std::string>{
        "leg_left_hip_joint", "leg_left_knee_joint",
        "leg_right_hip_joint", "leg_right_knee_joint"});
    wheel_joint_names_ = declare_parameter(
      "wheel_joint_names",
      std::vector<std::string>{"wheel_left_joint", "wheel_right_joint"});
    joint_kp_ = declare_parameter("joint_kp", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    joint_kd_ = declare_parameter("joint_kd", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    if (leg_joint_names_.size() != 4 || wheel_joint_names_.size() != 2) {
      throw std::invalid_argument("leg_joint_names 需 4 个、wheel_joint_names 需 2 个元素");
    }
    if (joint_kp_.size() != 4 || joint_kd_.size() != 4) {
      throw std::invalid_argument("joint_kp 与 joint_kd 各需 4 个元素");
    }

    // ---------------- 通信参数 ----------------
    command_timeout_ = declare_parameter("command_timeout", 0.3);
    send_rate_ = declare_parameter("send_rate", 100.0);
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));

    const std::string command_topic =
      declare_parameter("command_topic", std::string("leg_calc/control"));
    const std::string wheel_target_topic =
      declare_parameter("wheel_target_topic", std::string("wheel_target"));
    const std::string joint_target_topic =
      declare_parameter("joint_target_topic", std::string("joint_target"));
    const std::string odom_topic = declare_parameter("odom_topic", std::string("odom"));
    const std::string joint_states_topic =
      declare_parameter("joint_states_topic", std::string("joint_states"));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 10);
    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("usb_drive/state", 10);

    cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic, 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {onControl(*msg);});
    wheel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      wheel_target_topic, 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {onWheelTarget(*msg);});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_target_topic, 10,
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
    event_thread_ = std::thread([this]() {
      while (!exit_thread_) {
        cdc_->processOnce();
      }
    });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, send_rate_)),
      [this]() {sendTarget();});

    RCLCPP_INFO(
      get_logger(),
      "USB-CDC 通信节点启动: %04x:%04x 接口%d, 收发 %zu/%zu B, 发送 %.0f Hz, 轮子模式 %s",
      vid_, pid_, interface_number_, sizeof(usb_drive::RobotTarget_t),
      sizeof(usb_drive::RobotState_t), send_rate_, wheel_mode.c_str());
  }

  ~UsbDriveNode() override
  {
    exit_thread_ = true;
    if (event_thread_.joinable()) {
      event_thread_.join();
    }
    if (cdc_) {
      // 退出时下发“保持轮角 + 关节失力”的安全帧
      usb_drive::RobotTarget_t safe{};
      for (int i = 0; i < 2; ++i) {
        const bool invert = (i == 0) ? left_invert_ : right_invert_;
        const double dir = invert ? -1.0 : 1.0;
        safe.wheel[i].angle =
          static_cast<float>(dir * measured_wheel_angle_[static_cast<size_t>(i)]);
        safe.wheel[i].omega = 0.0f;
      }
      cdc_->sendStruct(safe);
      cdc_->close();
    }
  }

private:
  // ---------------- 下行：LQR 控制量 ----------------
  void onControl(const std_msgs::msg::Float64MultiArray & msg)
  {
    if (msg.data.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "收到空的控制指令，忽略");
      return;
    }
    command_u_ = msg.data[0];
    last_command_time_ = now();
    command_received_ = true;
  }

  // ---------------- 下行：差速底盘左右轮目标转速 ----------------
  void onWheelTarget(const std_msgs::msg::Float64MultiArray & msg)
  {
    if (msg.data.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "wheel_target 需要 2 个数 [左, 右]，忽略本帧");
      return;
    }
    wheel_target_speed_[0] = msg.data[0];
    wheel_target_speed_[1] = msg.data[1];
    last_wheel_target_time_ = now();
    wheel_target_received_ = true;
  }

  // ---------------- 下行：4 个腿关节目标 ----------------
  void onJointTarget(const sensor_msgs::msg::JointState & msg)
  {
    bool all_found = true;
    for (size_t i = 0; i < leg_joint_names_.size(); ++i) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), leg_joint_names_[i]);
      if (it == msg.name.end()) {
        all_found = false;
        break;
      }
      const size_t index = static_cast<size_t>(std::distance(msg.name.begin(), it));
      const float position =
        index < msg.position.size() ? static_cast<float>(msg.position[index]) : 0.0f;
      const float velocity =
        index < msg.velocity.size() ? static_cast<float>(msg.velocity[index]) : 0.0f;
      const float effort =
        index < msg.effort.size() ? static_cast<float>(msg.effort[index]) : 0.0f;
      joint_target_[i].angle = position;
      joint_target_[i].omega = velocity;
      joint_target_[i].torque = effort;
    }
    if (!all_found) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "joint_target 缺少关节名 %s 等，忽略本帧",
        leg_joint_names_[0].c_str());
      return;
    }
    last_joint_target_time_ = now();
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
    if (!plausibleState(state)) {
      // 失步：丢弃残余数据，等下一帧重新对齐
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "收到非法状态帧，清空接收缓冲区重新同步");
      assembler_->reset();
      return;
    }
    handleState(state);
  }

  void handleState(const usb_drive::RobotState_t & state)
  {
    const double dir_l = left_invert_ ? -1.0 : 1.0;
    const double dir_r = right_invert_ ? -1.0 : 1.0;

    const double angle_l = dir_l * state.wheel[0].angle;
    const double angle_r = dir_r * state.wheel[1].angle;
    const double omega_l = dir_l * state.wheel[0].omega;
    const double omega_r = dir_r * state.wheel[1].omega;

    measured_wheel_angle_[0] = angle_l;
    measured_wheel_angle_[1] = angle_r;

    const auto stamp = now();

    // 里程计：x = r*(φL+φR)/2
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = 0.5 * (angle_l + angle_r) * wheel_radius_;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = 0.5 * (omega_l + omega_r) * wheel_radius_;
    odom.twist.twist.angular.z =
      wheel_separation_ > 1e-9 ? (omega_r - omega_l) * wheel_radius_ / wheel_separation_ : 0.0;
    odom.pose.covariance[0] = 1e-3;
    odom.pose.covariance[7] = 1e-3;
    odom.pose.covariance[35] = 1e-2;
    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[35] = 1e-2;
    odom_pub_->publish(odom);

    // 关节状态：2 轮 + 4 关节
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

  // ---------------- 周期下发 ----------------
  void sendTarget()
  {
    const auto stamp = now();
    // 用实际发送间隔积分目标角，避免定时器抖动导致角度指令漂移
    const double dt = std::clamp((stamp - last_send_time_).seconds(), 1e-4, 0.1);
    last_send_time_ = stamp;
    usb_drive::RobotTarget_t target{};

    // 两路轮子指令：LQR 的平衡分量（u）与差速底盘的行驶分量（wheel_target）
    const bool cmd_fresh =
      command_received_ && (stamp - last_command_time_).seconds() <= command_timeout_;
    const bool diff_fresh =
      wheel_target_received_ && (stamp - last_wheel_target_time_).seconds() <= command_timeout_;
    const bool wheel_fresh = cmd_fresh || diff_fresh;

    if (command_received_ && !wheel_fresh && drive_active_) {
      RCLCPP_WARN(
        get_logger(), "控制指令中断超过 %.3f s，轮子改为位置保持、关节失力", command_timeout_);
      drive_active_ = false;
      wheel_angle_init_ = false;
    }
    if (wheel_fresh) {
      drive_active_ = true;
    }

    const double u = cmd_fresh ? std::clamp(command_u_, -max_command_, max_command_) : 0.0;

    for (size_t i = 0; i < 2; ++i) {
      const double gain = (i == 0) ? left_wheel_gain_ : right_wheel_gain_;
      const bool invert = (i == 0) ? left_invert_ : right_invert_;
      // 上行读到的角度/角速度已乘过 dir，写目标角时必须反着变换回固件方向
      const double dir = invert ? -1.0 : 1.0;
      double angle_cmd = measured_wheel_angle_[i];
      double omega_cmd = 0.0;

      if (wheel_fresh && wheel_cmd_mode_ == WheelCmdMode::kVelocity) {
        // 平衡分量左右同向，行驶分量左右不同 → 相加即“边走边平衡”
        double omega = cmd_fresh ? u * gain : 0.0;
        if (diff_fresh) {
          omega += wheel_target_speed_[i];
        }
        omega_cmd = std::clamp(omega, -max_wheel_omega_, max_wheel_omega_);
        if (!wheel_angle_init_) {
          // 首次进入闭环时从实测角度起积分，避免角度指令突变
          wheel_angle_target_[i] = measured_wheel_angle_[i];
          wheel_angle_init_ = true;
        }
        wheel_angle_target_[i] += omega_cmd * dt;
        angle_cmd = wheel_angle_target_[i];
      } else if (cmd_fresh && wheel_cmd_mode_ == WheelCmdMode::kPosition) {
        // 位置模式只认 u；差速分量（rad/s）在该模式下无意义，忽略
        angle_cmd = std::clamp(u * gain, -1e3, 1e3);
        omega_cmd = 0.0;
        wheel_angle_init_ = false;
      } else {
        // 无有效指令：保持当前角度
        angle_cmd = measured_wheel_angle_[i];
        omega_cmd = 0.0;
      }

      target.wheel[i].angle = static_cast<float>(dir * angle_cmd);
      target.wheel[i].omega = static_cast<float>(dir * omega_cmd);
    }

    const bool joint_fresh =
      joint_target_received_ && (stamp - last_joint_target_time_).seconds() <= command_timeout_;
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

    cdc_->sendStruct(target);
  }

  // CDC 参数
  uint16_t vid_{0x0483};
  uint16_t pid_{0x5740};
  int interface_number_{1};
  uint8_t ep_in_{0x81};
  uint8_t ep_out_{0x01};

  // 底盘参数
  double wheel_radius_{0.065};
  double wheel_separation_{0.20};
  double left_wheel_gain_{1.0};
  double right_wheel_gain_{1.0};
  bool left_invert_{false};
  bool right_invert_{false};
  double max_wheel_omega_{20.0};
  double max_command_{5.0};
  WheelCmdMode wheel_cmd_mode_{WheelCmdMode::kVelocity};

  // 关节参数
  std::vector<std::string> leg_joint_names_;
  std::vector<std::string> wheel_joint_names_;
  std::vector<double> joint_kp_;
  std::vector<double> joint_kd_;
  usb_drive::JointTarget_t joint_target_[4]{};

  // 通信参数
  double command_timeout_{0.3};
  double send_rate_{100.0};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  // 控制指令缓存
  double command_u_{0.0};
  bool command_received_{false};
  bool drive_active_{false};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_send_time_{0, 0, RCL_ROS_TIME};
  bool joint_target_received_{false};

  // 差速底盘轮速分量 [rad/s]
  double wheel_target_speed_[2]{0.0, 0.0};
  bool wheel_target_received_{false};
  rclcpp::Time last_wheel_target_time_{0, 0, RCL_ROS_TIME};

  // 轮子目标角度积分
  double wheel_angle_target_[2]{0.0, 0.0};
  bool wheel_angle_init_{false};
  double measured_wheel_angle_[2]{0.0, 0.0};

  // 接收帧重组（仅 USB 事件线程访问）
  std::unique_ptr<usb_drive::FrameAssembler> assembler_;

  std::unique_ptr<usb_drive::CdcTrans> cdc_;
  std::thread event_thread_;
  std::atomic_bool exit_thread_{false};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UsbDriveNode>());
  rclcpp::shutdown();
  return 0;
}
