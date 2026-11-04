/**
 * @brief 500 Hz 统一实时控制节点（ROS 外壳）
 *
 * 节点只做四件事，算法都在库里：
 *   ① 参数 ↔ ControllerConfig（周期边界切换）
 *   ② ROS 消息 ↔ StateSnapshot（leg_control_io）与 ControlOutput → 话题
 *   ③ 固定周期驱动 LegController::step()（leg_control_core）
 *   ④ 状态变化日志、周期抖动与最坏执行时间统计
 *
 * 数据流：
 *   /robot_command (外部指令)    ┐
 *   /imu/data                    ├─► 快照 ─► 状态机 ─► Pitch LQR ─► 双腿 VMC ─► 轮子分配
 *   /joint_states (2 轮 + 4 关节) │                                    │
 *   /odom                        ┘                                    ▼
 *                                    /joint_target (effort=τ) + /wheel_target (ω_L, ω_R)
 *
 * 调试话题：
 *   /leg_control/state  [state, faults, θ, θ̇, x, ẋ, u, h, 腿长, τ0..τ3, ω_L_ref, ω_R_ref]
 *   /leg_control/timing [周期均值, 周期最大, 执行均值, 执行最大 (ms), 超期次数]
 */

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Eigenvalues>

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "leg_calc/leg_control_core.hpp"
#include "leg_calc/leg_control_io.hpp"
#include "leg_calc/leg_control_types.hpp"
#include "leg_calc/param_utils.hpp"

namespace
{

/// 实时热路径（500 Hz 输入/输出）统一 QoS：只保留最新一帧 + best_effort。
///
/// 关键点不是“省带宽”，而是**去掉发布端的阻塞**：rmw_fastrtps 默认同步发送，
/// 可靠传输在订阅端（usb_drive 的单线程 executor 被 USB 同步写卡住时）会让
/// 队列堆满、publish() 原地等待，控制环随之停摆 ≥100 ms，于是 IMU / 关节 /
/// 轮反馈三条同时超时 → 故障字 0x23 停机。best_effort 下发布端只投递不等待。
rclcpp::QoS hotPathQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
}

}  // namespace

class LegControlNode : public rclcpp::Node
{
public:
  LegControlNode()
  : Node("leg_control")
  {
    declareParameters();

    const leg_calc::ControllerConfig cfg = buildConfig(nullptr);
    controller_.setConfig(cfg);
    logConfig(cfg);
    cacheInputConfig(cfg);
    loadRealtimeParams();

    // 三个互不干扰的回调组（配合 MultiThreadedExecutor）：
    //   ctrl_group_  ：500 Hz 控制环，独占一组，禁止与自身重入
    //   input_group_ ：IMU / joint_states / robot_command 回调，彼此串行（写同一份 io_）
    //   默认组        ：参数服务 + on_set_parameters 回调（rclcpp 固定放这里）
    // 这样参数校验里的 Eigen 求解、订阅回调里的解析都不会再把控制环饿死；
    // 换句话说，step() 的“最坏执行时间”从此只反映控制计算本身。
    ctrl_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions input_opts;
    input_opts.callback_group = input_group_;

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, hotPathQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {onImu(*msg);}, input_opts);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, hotPathQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJointStates(*msg);},
      input_opts);
    command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_, hotPathQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {onCommand(*msg);},
      input_opts);

    // 500 Hz 热路径输出：depth 1 + best_effort。
    // 可靠传输在订阅端（usb_drive）偶发卡顿时会让发布端原地阻塞，
    // 一个 100 ms 的阻塞就足以让 IMU/关节/轮反馈同时超时 → 0x23 停机。
    // 控制指令只需要“最新值”，丢帧由下一周期补上，因此宁可丢帧也不能阻塞。
    joint_target_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_target_topic_, hotPathQoS());
    wheel_target_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      wheel_target_topic_, hotPathQoS());
    wheel_torque_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      wheel_torque_topic_, hotPathQoS());
    // 调试话题（~100 Hz）保留 reliable：rviz2 / nav2 / topic echo 默认按可靠订阅
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("leg_control/state", 10);
    timing_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("leg_control/timing", 10);

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return onParameterChange(params);});

    last_cycle_time_ = now();
    last_timing_pub_ = now();
    last_debug_pub_ = now();
    last_state_log_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, cfg.control_rate)),
      [this]() {step();}, ctrl_group_);

    RCLCPP_INFO(
      get_logger(), "统一控制节点启动：%.0f Hz（调试输出 %.0f Hz），命令 %s，关节 %s → %s，"
      "轮速 %s，轮力矩 %s",
      cfg.control_rate, debug_rate_, command_topic_.c_str(), joint_states_topic_.c_str(),
      joint_target_topic_.c_str(), wheel_target_topic_.c_str(), wheel_torque_topic_.c_str());
  }

private:
  // ==================== 参数 ====================
  void declareParameters()
  {
    leg_calc::declareModelParams(*this, 0.085);
    leg_calc::declareLqrWeights(*this);

    declare_parameter("u_max", 5.0);
    declare_parameter("theta_axis", std::string("pitch"));
    declare_parameter("theta_offset", 0.0);
    declare_parameter("gain_mode", std::string("online_lqr"));
    declare_parameter("k_coeffs", std::vector<double>(12, 0.0));

    const leg_calc::LegGeometry leg_geometry;
    declare_parameter("thigh", leg_geometry.thigh);
    declare_parameter("shank", leg_geometry.shank);
    declare_parameter("com_offset", -0.032);
    declare_parameter("h_min", 0.06);
    declare_parameter("h_max", 0.12);
    declare_parameter("use_measured_height", true);
    declare_parameter("height_timeout", 0.50);

    const leg_calc::FiveBarGeometry fivebar;
    declare_parameter("l1", 0.050);
    declare_parameter("l2", 0.105);
    declare_parameter("l3", 0.105);
    declare_parameter("l4", 0.050);
    declare_parameter("l5", 0.060);
    declare_parameter("foot_branch", fivebar.foot_branch);
    declare_parameter("front_branch", fivebar.front_branch);
    declare_parameter("rear_branch", fivebar.rear_branch);

    const leg_calc::SpringDamperParams vmc;
    declare_parameter("kp_x", vmc.kp(0));
    declare_parameter("kp_y", vmc.kp(1));
    declare_parameter("kd_x", vmc.kd(0));
    declare_parameter("kd_y", vmc.kd(1));
    declare_parameter("force_max_x", vmc.force_max(0));
    declare_parameter("force_max_y", vmc.force_max(1));
    declare_parameter("vmc_gravity_comp", true);
    declare_parameter("foot_target_x", 0.03);

    declare_parameter("com_height_default", 0.085);
    declare_parameter("com_height_min", 0.065);
    declare_parameter("com_height_max", 0.100);
    declare_parameter("com_height_rate", 0.50);
    declare_parameter("joint_torque_max", 8.0);
    declare_parameter("joint_torque_sign", std::vector<double>{1.0, 1.0, 1.0, 1.0});
    declare_parameter("joint_sign", std::vector<double>{1.0, 1.0, 1.0, 1.0});
    declare_parameter("joint_offset", std::vector<double>{0.0, 0.0, 0.0, 0.0});

    declare_parameter("wheel_separation", 0.305);
    declare_parameter("max_wheel_omega", 20.0);
    declare_parameter("max_balance_omega", 8.0);
    declare_parameter("max_drive_omega", 12.0);
    declare_parameter("yaw_ff", 0.12);
    declare_parameter("k_vel_i", 0.0);
    declare_parameter("k_yaw_i", 0.60);
    declare_parameter("balance_torque_to_omega", 1.0);
    declare_parameter("balance_leak_rate", 6.0);

    // 轮子力矩通道（实机主通道，默认开；固件是纯力矩直通，关掉轮子就不出力）
    declare_parameter("wheel_torque_enable", true);
    declare_parameter("wheel_torque_max", 1.00);
    declare_parameter("k_vel_p", 1.0);
    declare_parameter("k_yaw_p", 1.20);
    declare_parameter("wheel_brake_damp", 0.02);

    declare_parameter("max_linear_velocity", 1.0);
    declare_parameter("max_yaw_rate", 1.0);
    declare_parameter("control_rate", 500.0);
    // 调试类输出（/odom + TF + /leg_control/state）的发布频率：控制环仍跑 control_rate，
    // 但这些话题 500 Hz 发布会把单线程 executor 和订阅端（rviz2/ros2 topic echo）拖垮，
    // 表现为“控制环最坏执行时间 xx ms 超过 2 ms 周期”。0 = 完全不发。
    declare_parameter("debug_rate", 100.0);

    // 控制线程调度优先级：先试 SCHED_FIFO（rt_priority），没权限（无 CAP_SYS_NICE、
    // 也没配 rtprio 限额）就退到 nice（rt_nice，0 = 不调）。两者都失败只告警一次。
    // 目的：2 ms 周期下，普通 CFS 线程会被同机编译/浏览器/IDE 抢 10 ms 量级的时间片。
    declare_parameter("rt_priority", 60);
    declare_parameter("rt_nice", -10);

    declare_parameter("imu_timeout", 0.10);
    declare_parameter("joint_state_timeout", 0.10);
    declare_parameter("wheel_state_timeout", 0.10);
    declare_parameter("command_timeout", 0.30);
    declare_parameter("command_fault_timeout", 1.00);
    declare_parameter("max_pitch", 0.60);
    declare_parameter("max_pitch_rate", 6.00);
    declare_parameter("startup_cycles", 250);

    declare_parameter(
      "leg_joint_names",
      std::vector<std::string>{
      "leg_left_hip_joint", "leg_left_knee_joint",
      "leg_right_hip_joint", "leg_right_knee_joint"});
    declare_parameter(
      "wheel_joint_names",
      std::vector<std::string>{"wheel_left_joint", "wheel_right_joint"});

    imu_topic_ = declare_parameter("imu_topic", std::string("imu/data"));
    odom_topic_ = declare_parameter("odom_topic", std::string("odom"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    joint_states_topic_ = declare_parameter("joint_states_topic", std::string("joint_states"));
    command_topic_ = declare_parameter("command_topic", std::string("robot_command"));
    joint_target_topic_ = declare_parameter("joint_target_topic", std::string("joint_target"));
    wheel_target_topic_ = declare_parameter("wheel_target_topic", std::string("wheel_target"));
    wheel_torque_topic_ = declare_parameter("wheel_torque_topic", std::string("wheel_torque"));
  }

  /// 读取 + 校验一组配置；pending 非空时优先使用参数回调里的新值
  leg_calc::ControllerConfig buildConfig(const leg_calc::ParamMap * pending)
  {
    auto get = [this, pending](const std::string & name) -> rclcpp::Parameter {
        return leg_calc::detail::lookupParam(*this, pending, name);
      };

    leg_calc::ControllerConfig cfg;
    cfg.model = leg_calc::readModelParams(*this, pending);
    leg_calc::readLqrWeights(*this, &cfg.Q, &cfg.R, pending);

    cfg.u_max = get("u_max").as_double();
    if (!(cfg.u_max > 0.0)) {
      throw std::invalid_argument("u_max 必须为正");
    }

    cfg.theta_axis = get("theta_axis").as_string();
    if (cfg.theta_axis != "pitch" && cfg.theta_axis != "roll") {
      throw std::invalid_argument("theta_axis 只能是 pitch 或 roll");
    }
    cfg.axis_index = (cfg.theta_axis == "roll") ? 0 : 1;
    cfg.theta_offset = get("theta_offset").as_double();

    const std::string gain_mode = get("gain_mode").as_string();
    if (gain_mode != "online_lqr" && gain_mode != "quadratic") {
      throw std::invalid_argument("gain_mode 只能是 online_lqr 或 quadratic");
    }
    cfg.gain_mode = (gain_mode == "online_lqr") ?
      leg_calc::GainMode::kOnlineLqr : leg_calc::GainMode::kQuadratic;
    const std::vector<double> k_coeffs = get("k_coeffs").as_double_array();
    if (k_coeffs.size() != 12) {
      throw std::invalid_argument("k_coeffs 必须包含 12 个数（4 个增益 x 3 个系数）");
    }
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        cfg.k_coeffs(i, j) = k_coeffs[static_cast<size_t>(i * 3 + j)];
      }
    }

    cfg.leg_geometry.thigh = get("thigh").as_double();
    cfg.leg_geometry.shank = get("shank").as_double();
    cfg.leg_geometry.com_offset = get("com_offset").as_double();
    cfg.h_min = get("h_min").as_double();
    cfg.h_max = get("h_max").as_double();
    cfg.use_measured_height = get("use_measured_height").as_bool();
    cfg.height_timeout = get("height_timeout").as_double();

    cfg.fivebar.l1 = get("l1").as_double();
    cfg.fivebar.l2 = get("l2").as_double();
    cfg.fivebar.l3 = get("l3").as_double();
    cfg.fivebar.l4 = get("l4").as_double();
    cfg.fivebar.l5 = get("l5").as_double();
    cfg.fivebar.foot_branch = static_cast<int>(get("foot_branch").as_int());
    cfg.fivebar.front_branch = static_cast<int>(get("front_branch").as_int());
    cfg.fivebar.rear_branch = static_cast<int>(get("rear_branch").as_int());
    if (!cfg.fivebar.isValid()) {
      throw std::invalid_argument("五杆几何非法：检查 l1..l5 与装配分支");
    }

    cfg.vmc.kp = Eigen::Vector2d(get("kp_x").as_double(), get("kp_y").as_double());
    cfg.vmc.kd = Eigen::Vector2d(get("kd_x").as_double(), get("kd_y").as_double());
    cfg.vmc.force_max =
      Eigen::Vector2d(get("force_max_x").as_double(), get("force_max_y").as_double());
    if ((cfg.vmc.kp.array() < 0.0).any() || (cfg.vmc.kd.array() < 0.0).any()) {
      throw std::invalid_argument("kp / kd 不能为负");
    }
    cfg.vmc_gravity_comp = get("vmc_gravity_comp").as_bool();
    cfg.foot_target_x = get("foot_target_x").as_double();

    cfg.com_height_default = get("com_height_default").as_double();
    cfg.com_height_min = get("com_height_min").as_double();
    cfg.com_height_max = get("com_height_max").as_double();
    cfg.com_height_rate = get("com_height_rate").as_double();
    if (!(cfg.com_height_min < cfg.com_height_default &&
      cfg.com_height_default <= cfg.com_height_max))
    {
      throw std::invalid_argument("质心高度必须满足 min < default <= max");
    }
    if (!(cfg.com_height_min > cfg.leg_geometry.com_offset)) {
      throw std::invalid_argument("com_height_min 必须大于 com_offset（否则换算出的腿长为负）");
    }
    for (int i = 0; i <= 10; ++i) {
      const double h = cfg.com_height_min +
        (cfg.com_height_max - cfg.com_height_min) * static_cast<double>(i) / 10.0;
      const Eigen::Vector2d foot(cfg.foot_target_x, h - cfg.leg_geometry.com_offset);
      Eigen::Vector2d joints;
      const double front_distance = foot.norm();
      const double rear_distance = (foot - cfg.fivebar.hipFar()).norm();
      if (!cfg.fivebar.inverseKinematics(foot, &joints) ||
        front_distance >= 0.90 * (cfg.fivebar.l1 + cfg.fivebar.l2) ||
        rear_distance >= 0.90 * (cfg.fivebar.l3 + cfg.fivebar.l4))
      {
        throw std::invalid_argument("目标质心高度超出五杆安全工作区间");
      }
    }
    cfg.joint_torque_max = get("joint_torque_max").as_double();
    if (!(cfg.joint_torque_max > 0.0)) {
      throw std::invalid_argument("joint_torque_max 必须为正");
    }
    const std::vector<double> sign = get("joint_torque_sign").as_double_array();
    if (sign.size() != 4) {
      throw std::invalid_argument("joint_torque_sign 必须包含 4 个数（±1）");
    }
    for (size_t i = 0; i < 4; ++i) {
      cfg.joint_torque_sign[i] = sign[i];
    }

    // 关节角 → 五杆角标定：α/β = joint_sign·q + joint_offset
    const std::vector<double> jsign = get("joint_sign").as_double_array();
    const std::vector<double> joffset = get("joint_offset").as_double_array();
    if (jsign.size() != 4 || joffset.size() != 4) {
      throw std::invalid_argument("joint_sign / joint_offset 必须各含 4 个数");
    }
    for (size_t i = 0; i < 4; ++i) {
      if (jsign[i] == 0.0) {
        throw std::invalid_argument("joint_sign 不能为 0");
      }
      cfg.joint_sign[i] = jsign[i];
      cfg.joint_offset[i] = joffset[i];
    }

    cfg.wheel_radius = cfg.model.wheel_radius;
    if (!(cfg.wheel_radius > 0.0)) {
      throw std::invalid_argument("wheel_radius 必须为正");
    }
    cfg.wheel_separation = get("wheel_separation").as_double();
    cfg.max_wheel_omega = get("max_wheel_omega").as_double();
    cfg.max_balance_omega = get("max_balance_omega").as_double();
    cfg.max_drive_omega = get("max_drive_omega").as_double();
    if (!std::isfinite(cfg.wheel_separation) || !(cfg.wheel_separation > 0.0)) {
      throw std::invalid_argument("wheel_separation 必须有限且为正");
    }
    if (!std::isfinite(cfg.max_drive_omega) || !(cfg.max_drive_omega > 0.0)) {
      throw std::invalid_argument("max_drive_omega 必须有限且为正");
    }
    cfg.yaw_ff = get("yaw_ff").as_double();
    if (!std::isfinite(cfg.yaw_ff) || cfg.yaw_ff < 0.0) {
      throw std::invalid_argument("yaw_ff 必须有限且非负");
    }
    cfg.k_vel_i = get("k_vel_i").as_double();
    cfg.k_yaw_i = get("k_yaw_i").as_double();
    if (!std::isfinite(cfg.k_vel_i) || cfg.k_vel_i < 0.0 ||
      !std::isfinite(cfg.k_yaw_i) || cfg.k_yaw_i < 0.0)
    {
      throw std::invalid_argument("k_vel_i / k_yaw_i 必须有限且非负");
    }
    cfg.balance_torque_to_omega = get("balance_torque_to_omega").as_double();
    cfg.balance_leak_rate = get("balance_leak_rate").as_double();
    if (!std::isfinite(cfg.balance_torque_to_omega) ||
      !(cfg.balance_torque_to_omega > 0.0) ||
      !(cfg.max_balance_omega > 0.0) || !(cfg.max_wheel_omega > 0.0))
    {
      throw std::invalid_argument("轮速映射系数和轮速限幅必须有限且为正");
    }

    // 轮子力矩通道（实机主通道，默认开）
    cfg.wheel_torque_enable = get("wheel_torque_enable").as_bool();
    cfg.wheel_torque_max = get("wheel_torque_max").as_double();
    cfg.k_vel_p = get("k_vel_p").as_double();
    cfg.k_yaw_p = get("k_yaw_p").as_double();
    cfg.wheel_brake_damp = get("wheel_brake_damp").as_double();
    if (!(cfg.wheel_torque_max > 0.0) || !std::isfinite(cfg.wheel_torque_max)) {
      throw std::invalid_argument("wheel_torque_max 必须有限且为正");
    }
    if (!std::isfinite(cfg.k_vel_p) || cfg.k_vel_p < 0.0 ||
      !std::isfinite(cfg.k_yaw_p) || cfg.k_yaw_p < 0.0 ||
      !std::isfinite(cfg.wheel_brake_damp) || cfg.wheel_brake_damp < 0.0)
    {
      throw std::invalid_argument("k_vel_p / k_yaw_p / wheel_brake_damp 必须有限且非负");
    }

    cfg.max_linear_velocity = get("max_linear_velocity").as_double();
    cfg.max_yaw_rate = get("max_yaw_rate").as_double();
    cfg.control_rate = get("control_rate").as_double();
    if (!(cfg.control_rate >= 50.0 && cfg.control_rate <= 2000.0)) {
      throw std::invalid_argument("control_rate 必须在 [50, 2000] Hz");
    }
    debug_rate_ = get("debug_rate").as_double();
    if (!std::isfinite(debug_rate_) || debug_rate_ < 0.0 || debug_rate_ > cfg.control_rate) {
      throw std::invalid_argument("debug_rate 必须在 [0, control_rate] 内（0 = 关掉调试输出）");
    }

    cfg.safety.imu_timeout = get("imu_timeout").as_double();
    cfg.safety.joint_timeout = get("joint_state_timeout").as_double();
    cfg.safety.wheel_timeout = get("wheel_state_timeout").as_double();
    cfg.safety.command_timeout = get("command_timeout").as_double();
    cfg.safety.command_fault_timeout = get("command_fault_timeout").as_double();
    cfg.safety.max_pitch = get("max_pitch").as_double();
    cfg.safety.max_pitch_rate = get("max_pitch_rate").as_double();
    cfg.safety.startup_cycles = static_cast<int>(get("startup_cycles").as_int());
    if (cfg.safety.command_fault_timeout < cfg.safety.command_timeout) {
      throw std::invalid_argument("command_fault_timeout 不能小于 command_timeout");
    }

    cfg.leg_joint_names = get("leg_joint_names").as_string_array();
    cfg.wheel_joint_names = get("wheel_joint_names").as_string_array();
    if (cfg.leg_joint_names.size() != 4 || cfg.wheel_joint_names.size() != 2) {
      throw std::invalid_argument("leg_joint_names 需 4 个、wheel_joint_names 需 2 个");
    }

    // 检查实际运行的 K(h)，包括二次拟合模式的闭环极点。
    leg_calc::GainSchedule probe(cfg.model, cfg.gain_mode);
    probe.setWeights(cfg.Q, cfg.R);
    probe.setHeightLimits(cfg.h_min, cfg.h_max);
    probe.setQuadraticCoefficients(cfg.k_coeffs);
    for (int i = 0; i <= 10; ++i) {
      const double h = cfg.h_min + (cfg.h_max - cfg.h_min) * static_cast<double>(i) / 10.0;
      const auto gain = probe.gainAt(h);
      if (!probe.lastSolveOk() || !gain.allFinite()) {
        throw std::invalid_argument("LQR 增益无效：请检查 Q/R、模型与 k_coeffs");
      }
      leg_calc::ModelParams params = cfg.model;
      params.com_length = h;
      leg_calc::WheelLegModel model(params);
      leg_calc::WheelLegModel::MatA A;
      leg_calc::WheelLegModel::MatB B;
      model.linearize(&A, &B);
      const Eigen::EigenSolver<leg_calc::WheelLegModel::MatA> poles(A - B * gain, false);
      if (poles.info() != Eigen::Success ||
        !poles.eigenvalues().allFinite() ||
        (poles.eigenvalues().real().array() >= -1e-6).any())
      {
        throw std::invalid_argument("实际 LQR 增益未稳定整个四状态模型");
      }
    }
    return cfg;
  }

  void logConfig(const leg_calc::ControllerConfig & cfg)
  {
    RCLCPP_INFO(
      get_logger(),
      "配置生效：u_max=%.2f N·m，质心高度 %.3f~%.3f m，关节力矩上限 %.2f N·m，"
      "平衡映射 %.2f rad/s/N·m，安全门限 [IMU %.2f s, 关节 %.2f s, 命令 %.2f/%.2f s]",
      cfg.u_max, cfg.com_height_min, cfg.com_height_max, cfg.joint_torque_max,
      cfg.balance_torque_to_omega, cfg.safety.imu_timeout, cfg.safety.joint_timeout,
      cfg.safety.command_timeout, cfg.safety.command_fault_timeout);
    RCLCPP_INFO(
      get_logger(),
      "轮子主通道：/wheel_torque（力矩直通，%.1f N·m/轮），轮距 %.3f m，"
      "平衡折算 %.2f rad/s/N·m，差速限幅 %.1f / 总限幅 %.1f rad/s（/wheel_target 仅仿真）",
      cfg.wheel_torque_max, cfg.wheel_separation, cfg.balance_torque_to_omega, cfg.max_drive_omega,
      cfg.max_wheel_omega);
    if (cfg.wheel_torque_enable) {
      RCLCPP_INFO(
        get_logger(),
        "轮子力矩通道：开（实机主通道），上限 %.2f N·m，k_vel_p=%.2f，k_yaw_p=%.2f，"
        "yaw_ff=%.2f，制动阻尼 %.3f",
        cfg.wheel_torque_max, cfg.k_vel_p, cfg.k_yaw_p, cfg.yaw_ff, cfg.wheel_brake_damp);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "轮子力矩通道：关（/wheel_torque 恒为 0）。固件是纯力矩直通，实机轮子将完全不出力");
    }
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    static const std::vector<std::string> kTunable = {
      "cart_mass", "pendulum_mass", "com_length", "com_inertia", "gravity",
      "wheel_radius", "wheel_inertia", "input_mode", "q_diag", "r_weight",
      "u_max", "theta_axis", "theta_offset", "gain_mode", "k_coeffs",
      "thigh", "shank", "com_offset", "h_min", "h_max",
      "use_measured_height", "height_timeout",
      "l1", "l2", "l3", "l4", "l5", "foot_branch", "front_branch", "rear_branch",
      "kp_x", "kp_y", "kd_x", "kd_y", "force_max_x", "force_max_y", "vmc_gravity_comp",
      "foot_target_x",
      "com_height_default", "com_height_min", "com_height_max", "com_height_rate",
      "joint_torque_max", "joint_torque_sign", "joint_sign", "joint_offset",
      "wheel_separation", "max_wheel_omega", "max_balance_omega", "max_drive_omega",
      "yaw_ff", "k_vel_i", "k_yaw_i",
      "balance_torque_to_omega", "balance_leak_rate",
      "wheel_torque_enable", "wheel_torque_max", "k_vel_p", "k_yaw_p", "wheel_brake_damp",
      "max_linear_velocity", "max_yaw_rate",
      "imu_timeout", "joint_state_timeout", "wheel_state_timeout",
      "command_timeout", "command_fault_timeout",
      "max_pitch", "max_pitch_rate", "startup_cycles", "leg_joint_names", "wheel_joint_names"};

    bool touched = false;
    bool rt_touched = false;
    for (const auto & param : params) {
      if (param.get_name() == "rt_priority" || param.get_name() == "rt_nice") {
        rt_touched = true;
        continue;
      }
      if (std::find(kTunable.begin(), kTunable.end(), param.get_name()) != kTunable.end()) {
        touched = true;
      }
    }
    // 调度优先级不属于 ControllerConfig：改完只置脏标志，由控制环重新应用到
    // “当前真正执行 step() 的那个线程”（MultiThreadedExecutor 可能换线程）
    if (rt_touched) {
      rt_priority_ = static_cast<int>(get_parameter("rt_priority").as_int());
      rt_nice_ = static_cast<int>(get_parameter("rt_nice").as_int());
      rt_dirty_ = true;
    }
    if (!touched) {
      return result;
    }

    leg_calc::ParamMap pending;
    for (const auto & param : params) {
      pending[param.get_name()] = param;
    }

    try {
      // 只校验 + 缓存，真正的切换放到下一个控制周期边界
      const leg_calc::ControllerConfig next = buildConfig(&pending);
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_config_ = next;
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = e.what();
    }
    return result;
  }

  /// 把订阅回调需要的那几个配置项拷一份出来：回调线程不再直接读 controller_，
  /// 避免 setConfig() 与 config() 跨线程竞争
  void cacheInputConfig(const leg_calc::ControllerConfig & cfg)
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    input_axis_index_ = cfg.axis_index;
    input_theta_offset_ = cfg.theta_offset;
    input_leg_joint_names_ = cfg.leg_joint_names;
    input_wheel_joint_names_ = cfg.wheel_joint_names;
  }

  /// 参数里的 rt_priority / rt_nice 读进原子量（构造与参数回调都会走）
  void loadRealtimeParams()
  {
    rt_priority_ = static_cast<int>(get_parameter("rt_priority").as_int());
    rt_nice_ = static_cast<int>(get_parameter("rt_nice").as_int());
    rt_dirty_ = true;
  }

  /// 保证“正在执行 step() 的这个线程”拿到了实时优先级。
  ///
  /// MultiThreadedExecutor 不保证总用同一个线程跑定时器回调（实测在 3 个线程间
  /// 来回跳），所以按线程 id 记账：每个线程只尝试设置一次，设置过的直接跳过。
  /// 这样既不会每周期重复调用 pthread_setschedparam，也不会把日志刷爆。
  void ensureRealtime()
  {
    if (rt_dirty_.exchange(false)) {
      rt_threads_.clear();
      rt_warned_ = false;
    }
    const std::thread::id tid = std::this_thread::get_id();
    if (std::find(rt_threads_.begin(), rt_threads_.end(), tid) != rt_threads_.end()) {
      return;
    }
    // executor 线程数有限，这里只是防御性上限（不会真的超过线程池大小）
    if (rt_threads_.size() >= 8) {
      rt_threads_.clear();
      rt_warned_ = false;
    }
    rt_threads_.push_back(tid);
    applyRealtimePolicy();
  }

  void applyRealtimePolicy()
  {
    const int priority = rt_priority_.load();
    const int nice = rt_nice_.load();
    if (priority <= 0 && nice == 0) {
      return;
    }

    // ① SCHED_FIFO（需要 CAP_SYS_NICE，或 /etc/security/limits.conf 里给 rtprio）
    //   注意：pthread_* 直接返回错误码，不写 errno
    int fifo_err = 0;
    if (priority > 0) {
      sched_param sp{};
      sp.sched_priority = priority;
      fifo_err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
      if (fifo_err == 0) {
        RCLCPP_INFO(get_logger(), "控制线程已切到 SCHED_FIFO 优先级 %d", priority);
        return;
      }
    }

    // ② 退化到 nice（Linux 上 nice 是 per-thread 的，只影响这个线程）
    int nice_err = 0;
    if (nice != 0) {
      if (setpriority(PRIO_PROCESS, 0, nice) == 0) {
        RCLCPP_INFO(
          get_logger(), "控制线程 nice=%d 生效（SCHED_FIFO 不可用：%s）",
          nice, fifo_err != 0 ? std::strerror(fifo_err) : "未启用");
        return;
      }
      nice_err = errno;
    }

    // 两条都失败：只告警一次，别把 500 Hz 的日志刷爆
    if (!rt_warned_) {
      rt_warned_ = true;
      RCLCPP_WARN(
        get_logger(),
        "无法提升控制线程优先级（SCHED_FIFO %d: %s；nice %d: %s）：2 ms 周期下可能出现"
        "十几毫秒的调度抖动。修法：sudo setcap cap_sys_nice+ep install/leg_calc/lib/"
        "leg_calc/leg_control_node（或 limits.conf 加 \"<user> - rtprio 80\"），"
        "并把 CPU governor 设为 performance",
        priority, fifo_err != 0 ? std::strerror(fifo_err) : "未启用",
        nice, nice_err != 0 ? std::strerror(nice_err) : "未启用");
    }
  }

  // ==================== 输入回调 ====================
  // 这些回调跑在 input_group_ 的线程上，与 step() 并发；
  // 它们只碰 io_ 和“配置快照副本”，不读 controller_，因此 setConfig 不必加锁。
  void onImu(const sensor_msgs::msg::Imu & msg)
  {
    const double stamp_s = now().seconds();
    std::lock_guard<std::mutex> lock(io_mutex_);
    io_.onImu(msg, input_axis_index_, input_theta_offset_, stamp_s);
  }

  void onJointStates(const sensor_msgs::msg::JointState & msg)
  {
    const double stamp_s = now().seconds();
    std::lock_guard<std::mutex> lock(io_mutex_);
    io_.onJointStates(msg, input_leg_joint_names_, input_wheel_joint_names_, stamp_s);
  }

  void onCommand(const std_msgs::msg::Float64MultiArray & msg)
  {
    if (msg.data.size() < static_cast<size_t>(leg_calc::kCmdCount)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "/%s 需要 %zu 个数，忽略本帧",
        command_topic_.c_str(), static_cast<size_t>(leg_calc::kCmdCount));
      return;
    }
    const double stamp_s = now().seconds();
    std::lock_guard<std::mutex> lock(io_mutex_);
    io_.onCommand(msg, stamp_s);
  }

  // ==================== 500 Hz 主循环 ====================
  void step()
  {
    const auto cycle_start = std::chrono::steady_clock::now();
    ensureRealtime();
    const rclcpp::Time stamp = now();
    const double stamp_s = stamp.seconds();

    // 1) 参数在周期边界统一切换（与参数服务线程共享 pending_config_）
    std::optional<leg_calc::ControllerConfig> pending;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending.swap(pending_config_);
    }
    if (pending) {
      controller_.setConfig(*pending);
      cacheInputConfig(*pending);
      logConfig(*pending);
    }

    const double dt = std::clamp((stamp - last_cycle_time_).seconds(), 1e-4, 0.05);
    last_cycle_time_ = stamp;

    // 2) 快照 → 控制计算（io_ 只在锁内碰，临界区是几次赋值）
    leg_calc::StateSnapshot snapshot;
    double travel = 0.0;
    {
      std::lock_guard<std::mutex> lock(io_mutex_);
      travel = io_.wheelTravel(controller_.config().wheel_radius);
      snapshot = io_.build(stamp_s, controller_.config().wheel_radius);
    }
    const leg_calc::ControlOutput output = controller_.step(snapshot, dt);

    // 3) 输出
    joint_target_pub_->publish(
      leg_calc::makeJointTarget(
        controller_.config().leg_joint_names, snapshot, output, stamp));
    // /wheel_target 只给仿真桥与调试观测（实机固件不读 angle/omega）
    wheel_target_pub_->publish(leg_calc::makeWheelTarget(output));
    // 实机主通道：/wheel_torque → 帧里 wheel[].torque → 固件乘 WHEEL_COUNTS_PER_NM 变电流
    const leg_calc::ControllerConfig & cfg_now = controller_.config();
    const double force_to_torque =
      (cfg_now.model.input_mode == leg_calc::InputMode::kForce) ?
      cfg_now.model.wheel_radius : 1.0;
    wheel_torque_pub_->publish(leg_calc::makeWheelTorque(output, force_to_torque));

    // 调试类输出降频：里程计积分仍每周期做（丢周期会丢弧长），只把话题/TF 按 debug_rate 发
    const bool debug_tick = debugDue(stamp);
    publishOdom(snapshot, travel, stamp, debug_tick);
    if (debug_tick) {
      state_pub_->publish(leg_calc::makeStateDebug(snapshot, output));
    }

    reportStateChange(output);

    // 4) 实时性统计
    const auto cycle_end = std::chrono::steady_clock::now();
    const double execution_ms =
      std::chrono::duration<double, std::milli>(cycle_end - cycle_start).count();
    const double period_ms = dt * 1000.0;
    ++timing_count_;
    period_sum_ms_ += period_ms;
    exec_sum_ms_ += execution_ms;
    period_max_ms_ = std::max(period_max_ms_, period_ms);
    exec_max_ms_ = std::max(exec_max_ms_, execution_ms);
    if (period_ms > 1.5 * 1000.0 / controller_.config().control_rate) {
      ++overrun_count_;
    }
    publishTiming(stamp);
  }

  /// 调试输出是否该发：按 debug_rate 降频（0 = 不输出）
  bool debugDue(const rclcpp::Time & stamp)
  {
    if (debug_rate_ <= 0.0) {
      return false;
    }
    if ((stamp - last_debug_pub_).seconds() < 1.0 / debug_rate_) {
      return false;
    }
    last_debug_pub_ = stamp;
    return true;
  }

  /// 2D 轮式里程计：前进弧长（轮角差分）× 朝向（IMU yaw）→ odom 位姿 + odom→base_link TF
  /// travel 由 step() 在 io_mutex_ 内取出后传进来（io_ 不再在锁外被读）
  void publishOdom(
    const leg_calc::StateSnapshot & snapshot, double travel, const rclcpp::Time & stamp,
    bool publish)
  {
    // 朝向取 IMU yaw，并减掉启动时的初始朝向（使起步朝 +x）
    if (snapshot.imu_ok && !yaw_offset_set_) {
      yaw_offset_ = snapshot.yaw;
      yaw_offset_set_ = true;
    }
    const double heading = snapshot.yaw - yaw_offset_;

    // 前进弧长用绝对轮角差分（比 v·dt 抗速度抖动，不累积速度误差）
    double ds = 0.0;
    if (last_wheel_travel_valid_) {
      ds = travel - last_wheel_travel_;
    }
    last_wheel_travel_ = travel;
    last_wheel_travel_valid_ = true;

    odom_x_ += ds * std::cos(heading);
    odom_y_ += ds * std::sin(heading);

    if (!publish) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    // yaw-only 四元数（roll/pitch 归零，符合 2D 里程计惯例）
    odom.pose.pose.orientation.z = std::sin(heading * 0.5);
    odom.pose.pose.orientation.w = std::cos(heading * 0.5);
    odom.twist.twist.linear.x = snapshot.x_dot;
    odom.twist.twist.angular.z = snapshot.yaw_rate;
    odom.pose.covariance[0] = 1e-3;
    odom.pose.covariance[7] = 1e-3;
    odom.pose.covariance[35] = 1e-2;
    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[35] = 1e-2;
    odom_pub_->publish(odom);

    // odom → base_link 的 TF，供 rviz2 显示机器人位姿
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = odom_x_;
    tf.transform.translation.y = odom_y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  void publishTiming(const rclcpp::Time & stamp)
  {
    if ((stamp - last_timing_pub_).seconds() < 1.0 || timing_count_ == 0) {
      return;
    }
    std_msgs::msg::Float64MultiArray msg;
    const double n = static_cast<double>(timing_count_);
    msg.data = {
      period_sum_ms_ / n, period_max_ms_, exec_sum_ms_ / n, exec_max_ms_,
      static_cast<double>(overrun_count_)};
    timing_pub_->publish(msg);

    if (exec_max_ms_ > 2.0) {
      RCLCPP_WARN(
        get_logger(), "控制环最坏执行时间 %.2f ms 超过 2 ms 周期，请检查 CPU 负载",
        exec_max_ms_);
    }

    last_timing_pub_ = stamp;
    timing_count_ = 0;
    period_sum_ms_ = 0.0;
    exec_sum_ms_ = 0.0;
    period_max_ms_ = 0.0;
    exec_max_ms_ = 0.0;
    overrun_count_ = 0;
  }

  void reportStateChange(const leg_calc::ControlOutput & output)
  {
    if (state_reported_ && output.state == last_reported_state_ &&
      output.faults == last_reported_faults_)
    {
      return;
    }
    // 抖动抑制：故障位每周期都可能翻，日志一多（尤其终端是 IDE pipe）会反压控制环
    const rclcpp::Time log_stamp = now();
    if ((log_stamp - last_state_log_).seconds() >= 0.2) {
      last_state_log_ = log_stamp;
      if (output.state == leg_calc::ControlState::kFault) {
        RCLCPP_ERROR(
          get_logger(), "状态 %s，故障 0x%02X",
          leg_calc::toString(output.state), output.faults);
      } else {
        RCLCPP_INFO(
          get_logger(), "状态 %s，故障 0x%02X",
          leg_calc::toString(output.state), output.faults);
      }
    }
    last_reported_state_ = output.state;
    last_reported_faults_ = output.faults;
    state_reported_ = true;
  }

  // ==================== 成员 ====================
  leg_calc::LegController controller_{};
  /// io_ 与“订阅回调用的配置副本”都受这把锁保护：
  /// 输入回调跑在 input_group_ 线程，step() 跑在 ctrl_group_ 线程
  leg_calc::SnapshotIO io_{};
  std::mutex io_mutex_;
  int input_axis_index_{0};
  double input_theta_offset_{0.0};
  std::vector<std::string> input_leg_joint_names_;
  std::vector<std::string> input_wheel_joint_names_;

  /// 参数回调（默认回调组线程）→ 控制环（ctrl_group_ 线程）
  std::optional<leg_calc::ControllerConfig> pending_config_{};
  std::mutex pending_mutex_;

  // 控制线程实时性（rt_threads_ / rt_warned_ 只由 step() 访问：
  // 定时器独占 ctrl_group_，不会重入）
  std::atomic<int> rt_priority_{0};
  std::atomic<int> rt_nice_{0};
  std::atomic_bool rt_dirty_{true};
  std::vector<std::thread::id> rt_threads_;
  bool rt_warned_{false};

  std::string imu_topic_{"imu/data"};
  std::string odom_topic_{"odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string joint_states_topic_{"joint_states"};
  std::string command_topic_{"robot_command"};
  std::string joint_target_topic_{"joint_target"};
  std::string wheel_target_topic_{"wheel_target"};
  std::string wheel_torque_topic_{"wheel_torque"};

  rclcpp::Time last_cycle_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_timing_pub_{0, 0, RCL_ROS_TIME};
  /// 调试输出降频：最后一次发布 /odom、TF、/leg_control/state 的时间
  rclcpp::Time last_debug_pub_{0, 0, RCL_ROS_TIME};
  double debug_rate_{100.0};
  /// 状态变化日志的最小间隔，避免故障抖动时把终端刷爆（日志阻塞也会反压控制环）
  rclcpp::Time last_state_log_{0, 0, RCL_ROS_TIME};
  int timing_count_{0};
  double period_sum_ms_{0.0};
  double period_max_ms_{0.0};
  double exec_sum_ms_{0.0};
  double exec_max_ms_{0.0};
  int overrun_count_{0};

  // 2D 里程计积分状态（x/y 由前进弧长沿 IMU yaw 分解）
  double odom_x_{0.0};
  double odom_y_{0.0};
  double yaw_offset_{0.0};
  bool yaw_offset_set_{false};
  double last_wheel_travel_{0.0};
  bool last_wheel_travel_valid_{false};

  bool state_reported_{false};
  leg_calc::ControlState last_reported_state_{leg_calc::ControlState::kInit};
  uint32_t last_reported_faults_{leg_calc::kFaultNone};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_torque_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::CallbackGroup::SharedPtr ctrl_group_;
  rclcpp::CallbackGroup::SharedPtr input_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 多线程 executor：控制环（ctrl_group_）、输入订阅（input_group_）、参数服务（默认组）
  // 三者互不相同的 callback group，可以并行推进 —— 单个回调再慢也不会把控制环饿死。
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  auto node = std::make_shared<LegControlNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
