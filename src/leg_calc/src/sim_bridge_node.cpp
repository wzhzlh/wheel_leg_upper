/**
 * @brief 仿真 I/O 桥（仅仿真用）：把 ros2_control 的接口接到 leg_calc 实机链路
 *
 * 两个方向：
 *   ① 传感器方向
 *        /imu_site_imu_sensor/imu  (sensor_msgs/Imu，mujoco 插件只填了 gyro + accel，
 *                                   orientation 恒为 0，leg_control 直接拿去算 rpy 会得到 NaN)
 *        /base_link_pose_sensor/pose (geometry_msgs/PoseStamped，含 base_link 世界姿态)
 *                    ↓ 合并
 *        /imu/data                 (sensor_msgs/Imu，orientation 由 pose 补上)
 *
 *   ② 指令方向
 *        /joint_target  (JointState，effort = 4 个髋关节力矩)
 *            ↓ 按名字重排 / 限幅
 *        /leg_effort_controller/commands   (Float64MultiArray)
 *
 *        /wheel_target  (Float64MultiArray，[ω_L_ref, ω_R_ref])
 *        /joint_states  (实测轮速)
 *            ↓ τ = kp · (ω_ref − ω) + ki · ∫(ω_ref − ω)dt
 *        /wheel_effort_controller/commands (Float64MultiArray)
 *
 * 说明：
 *   * 仿真里的轮子没有下位机速度环，只能用力矩闭环跟上 /wheel_target 的轮速目标。
 *     轮子惯量极小（J≈3e-5 kg·m²），纯 P 有硬上限 kp < 2J/Δt ≈ 0.03，超过就抖；
 *     而靠 P 顶不住速度（力矩太小），所以要加积分项当“硬速度环”用。
 *   * mujoco 的 gyro 是 site 局部系角速度，而 leg_control 拿 angular_velocity.y/.z
 *     当 pitch_rate / yaw_rate，所以这里把角速度转到世界系，和发出去的姿态保持一致。
 *   * /joint_target 超过 command_timeout 没有更新时，两路命令都清零，
 *     避免仿真继续吃陈旧力矩（JointGroupEffortController 会一直保持最后一条命令）。
 */

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace
{

/// 用四元数 (w,x,y,z) 把向量从 body 系转到世界系
std::array<double, 3> rotateByQuat(
  const std::array<double, 4> & q, const std::array<double, 3> & v)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double R[9] = {
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
    2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
    2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)
  };
  return {
    R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
    R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
    R[6] * v[0] + R[7] * v[1] + R[8] * v[2]
  };
}

/// 四元数乘法 q1 ⊗ q2（Hamilton 积），输入输出均为 (w,x,y,z)
std::array<double, 4> mulQuat(
  const std::array<double, 4> & a, const std::array<double, 4> & b)
{
  return {
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
  };
}

}  // namespace

class SimBridge : public rclcpp::Node
{
public:
  SimBridge()
  : Node("sim_bridge")
  {
    // ---------------- 话题 ----------------
    imu_raw_topic_ = declare_parameter("imu_raw_topic", std::string("imu_site_imu_sensor/imu"));
    pose_topic_ = declare_parameter("pose_topic", std::string("base_link_pose_sensor/pose"));
    imu_out_topic_ = declare_parameter("imu_out_topic", std::string("imu/data"));
    joint_target_topic_ = declare_parameter("joint_target_topic", std::string("joint_target"));
    wheel_target_topic_ = declare_parameter("wheel_target_topic", std::string("wheel_target"));
    joint_states_topic_ = declare_parameter("joint_states_topic", std::string("joint_states"));
    leg_cmd_topic_ =
      declare_parameter("leg_cmd_topic", std::string("leg_effort_controller/commands"));
    wheel_cmd_topic_ =
      declare_parameter("wheel_cmd_topic", std::string("wheel_effort_controller/commands"));

    // ---------------- 频率 ----------------
    imu_rate_ = declare_parameter("imu_rate", 200.0);
    cmd_rate_ = declare_parameter("cmd_rate", 500.0);
    sensor_timeout_ = declare_parameter("sensor_timeout", 0.10);

    // ---------------- 桥接参数 ----------------
    wheel_kp_ = declare_parameter("wheel_kp", 0.02);
    wheel_ki_ = declare_parameter("wheel_ki", 2.0);
    wheel_tau_max_ = declare_parameter("wheel_tau_max", 3.0);
    // 轮子指令解释方式：torque（把 /wheel_target 当成力矩，再乘 gain）/ speed（PI 跟踪轮速）
    wheel_mode_ = declare_parameter("wheel_mode", std::string("speed"));
    wheel_torque_gain_ = declare_parameter("wheel_torque_gain", 1.0);
    // 启动门控期间机身被夹住，轮子若无阻力会瞬间空转（力矩模式尤其明显）。
    // 用仿真时间（pose 头时间戳）做延时，门控放开后再真正下发轮子力矩 [s]
    wheel_start_delay_ = declare_parameter("wheel_start_delay", 1.5);
    wheel_arm_on_gate_ = declare_parameter("wheel_arm_on_gate", true);
    leg_torque_max_ = declare_parameter("leg_torque_max", 12.0);
    command_timeout_ = declare_parameter("command_timeout", 0.25);
    // base_link 的坐标轴与实机 IMU 不一定同向；若仿真里机器人一使能就反向加速，
    // 把这个打开（等价于把 IMU 翻过来装：roll / pitch 取反，yaw 不变）
    flip_roll_pitch_ = declare_parameter("flip_roll_pitch", false);
    leg_joint_order_ = declare_parameter(
      "leg_joint_order",
      std::vector<std::string>{"lf_joint1", "lb_joint1", "rf_joint1", "rb_joint1"});
    wheel_joint_order_ = declare_parameter(
      "wheel_joint_order", std::vector<std::string>{"lefoot_joint", "rifoot_joint"});

    // ---------------- 传感器 ----------------
    imu_raw_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_raw_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {onImuRaw(*msg);});
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {onPose(*msg);});

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_out_topic_, rclcpp::SensorDataQoS());

    // ---------------- 指令 ----------------
    joint_target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_target_topic_, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJointTarget(*msg);});
    wheel_target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      wheel_target_topic_, 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {onWheelTarget(*msg);});
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJointStates(*msg);});

    // 启动门控状态（latched）：门控放开前不给轮子下发力矩，避免"挂架夹着、轮子空转"，
    // 放开瞬间被转动惯量弹飞（会直接触发 leg_control 的 kFaultAttitude）。
    rclcpp::QoS gate_qos(1);
    gate_qos.transient_local().reliable();
    start_gate_sub_ = create_subscription<std_msgs::msg::Bool>(
      "mujoco/start_gate_open", gate_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        gate_seen_ = true;
        gate_open_ = msg->data;
      });

    leg_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(leg_cmd_topic_, 10);
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(wheel_cmd_topic_, 10);

    // ---------------- 定时器 ----------------
    imu_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, imu_rate_)),
      [this]() {publishImu();});
    cmd_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_)),
      [this]() {publishCommands();});

    last_ctrl_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "仿真桥接启动：%s + %s → %s（%.0f Hz）；%s + %s → %s / %s（%.0f Hz，轮子 kp=%.2f）",
      imu_raw_topic_.c_str(), pose_topic_.c_str(), imu_out_topic_.c_str(), imu_rate_,
      joint_target_topic_.c_str(), wheel_target_topic_.c_str(),
      leg_cmd_topic_.c_str(), wheel_cmd_topic_.c_str(), cmd_rate_, wheel_kp_);
  }

private:
  // ==================== 传感器回调 ====================
  void onImuRaw(const sensor_msgs::msg::Imu & msg)
  {
    if (!std::isfinite(msg.angular_velocity.x) ||
      !std::isfinite(msg.angular_velocity.y) ||
      !std::isfinite(msg.angular_velocity.z))
    {
      return;
    }
    gyro_ = {msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
    accel_ = {msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z};
    imu_frame_ = msg.header.frame_id;
    imu_raw_seen_ = true;
    last_imu_receive_time_ = now();
  }

  void onPose(const geometry_msgs::msg::PoseStamped & msg)
  {
    const auto & orientation = msg.pose.orientation;
    const double norm2 = orientation.w * orientation.w + orientation.x * orientation.x +
      orientation.y * orientation.y + orientation.z * orientation.z;
    if (!std::isfinite(norm2) || norm2 < 1e-12) {
      return;
    }
    // MuJoCo framequat 输出 (w, x, y, z)，pose_sensor 已经映射到 geometry_msgs 的分量顺序
    quat_ = {msg.pose.orientation.w, msg.pose.orientation.x,
      msg.pose.orientation.y, msg.pose.orientation.z};
    pose_seen_ = true;
    last_pose_receive_time_ = now();
    pose_time_ = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    if (!first_pose_time_set_) {
      first_pose_time_ = pose_time_;
      first_pose_time_set_ = true;
    }
  }

  /// 用最新缓存合成一条完整 Imu（gyro + accel + orientation）
  void publishImu()
  {
    if (!imu_raw_seen_ || !pose_seen_ ||
      (now() - last_imu_receive_time_).seconds() > sensor_timeout_ ||
      (now() - last_pose_receive_time_).seconds() > sensor_timeout_)
    {
      return;
    }
    sensor_msgs::msg::Imu out;
    out.header.stamp = now();
    out.header.frame_id = imu_frame_;

    // MuJoCo 的 gyro 是 site 局部系（与 base_link 同向）的角速度，转到世界系后
    // .y 就是俯仰角速度（世界 y 是轮轴/俯仰轴），leg_control 直接取 .y 当 pitch_rate。
    const std::array<double, 3> omega_body = {gyro_[0], gyro_[1], gyro_[2]};
    std::array<double, 3> omega_world = rotateByQuat(quat_, omega_body);

    // 姿态：base_link 的机体系是 "x 前 / y 下 / z 左"，直接解 rpy 时静止姿态
    // roll≈-90°（ZYX 退化区），俯仰信息会跑到 yaw 上。先绕 x 转 +90°，换成
    // "x 前 / y 左 / z 上" 的 z-up 机体系，静止时 rpy=(0,0,0)，rpy[1] 才是真俯仰。
    const std::array<double, 4> q_x90 = {0.7071067811865476, 0.7071067811865476, 0.0, 0.0};
    std::array<double, 4> q = mulQuat(quat_, q_x90);
    if (flip_roll_pitch_) {
      // 等价于把 IMU 翻过来装：与 Rz(π) 共轭，roll / pitch 反号、yaw 不变
      q[1] = -q[1];
      q[2] = -q[2];
      omega_world[0] = -omega_world[0];
      omega_world[1] = -omega_world[1];
    }
    out.orientation.w = q[0];
    out.orientation.x = q[1];
    out.orientation.y = q[2];
    out.orientation.z = q[3];

    out.angular_velocity.x = omega_world[0];
    out.angular_velocity.y = omega_world[1];
    out.angular_velocity.z = omega_world[2];
    out.linear_acceleration.x = accel_[0];
    out.linear_acceleration.y = accel_[1];
    out.linear_acceleration.z = accel_[2];

    imu_pub_->publish(out);
  }

  // ==================== 指令回调 ====================
  void onJointTarget(const sensor_msgs::msg::JointState & msg)
  {
    leg_effort_.fill(0.0);
    for (size_t i = 0; i < leg_joint_order_.size() && i < 4; ++i) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), leg_joint_order_[i]);
      if (it == msg.name.end()) {
        continue;
      }
      const size_t idx = static_cast<size_t>(std::distance(msg.name.begin(), it));
      if (idx < msg.effort.size()) {
        leg_effort_[i] = std::clamp(msg.effort[idx], -leg_torque_max_, leg_torque_max_);
      }
    }
    last_ctrl_time_ = now();
    leg_target_seen_ = true;
  }

  void onWheelTarget(const std_msgs::msg::Float64MultiArray & msg)
  {
    if (msg.data.size() < 2) {
      return;
    }
    wheel_ref_ = {msg.data[0], msg.data[1]};
    last_ctrl_time_ = now();
    wheel_target_seen_ = true;
  }

  void onJointStates(const sensor_msgs::msg::JointState & msg)
  {
    for (size_t i = 0; i < wheel_joint_order_.size() && i < 2; ++i) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), wheel_joint_order_[i]);
      if (it == msg.name.end()) {
        continue;
      }
      const size_t idx = static_cast<size_t>(std::distance(msg.name.begin(), it));
      if (idx < msg.velocity.size()) {
        wheel_vel_[i] = msg.velocity[idx];
      }
    }
    wheel_meas_seen_ = true;
  }

  // ==================== 指令下发 ====================
  void publishCommands()
  {
    const bool fresh = leg_target_seen_ && wheel_target_seen_ &&
      (now() - last_ctrl_time_).seconds() <= command_timeout_;
    const double dt = 1.0 / std::max(1.0, cmd_rate_);
    const bool wheel_armed = (wheel_arm_on_gate_ && gate_seen_)
      ? gate_open_
      : (first_pose_time_set_ && (pose_time_ - first_pose_time_) >= wheel_start_delay_);

    std_msgs::msg::Float64MultiArray leg_cmd;
    leg_cmd.data.assign(4, 0.0);
    if (fresh) {
      leg_cmd.data.assign(leg_effort_.begin(), leg_effort_.end());
    }
    leg_cmd_pub_->publish(leg_cmd);

    std_msgs::msg::Float64MultiArray wheel_cmd;
    wheel_cmd.data.assign(2, 0.0);
    if (fresh && wheel_meas_seen_ && wheel_armed) {
      for (size_t i = 0; i < 2; ++i) {
        if (wheel_mode_ == "torque") {
          // /wheel_target 直接当作轮轴力矩（leg_control 的 LQR 本来就是力矩输出）
          wheel_cmd.data[i] = std::clamp(
            wheel_ref_[i] * wheel_torque_gain_, -wheel_tau_max_, wheel_tau_max_);
          continue;
        }
        const double err = wheel_ref_[i] - wheel_vel_[i];
        // 积分项：轮子很轻（J≈3e-5），纯 P 想“顶住”速度只能给很小的力矩，
        // 必须靠积分把力矩垫上去，才能等效实机 M2006 的硬速度环
        wheel_i_[i] = std::clamp(
          wheel_i_[i] + wheel_ki_ * err * dt, -wheel_tau_max_, wheel_tau_max_);
        const double tau = wheel_kp_ * err + wheel_i_[i];
        wheel_cmd.data[i] = std::clamp(tau, -wheel_tau_max_, wheel_tau_max_);
      }
    } else {
      wheel_i_.fill(0.0);
    }
    wheel_cmd_pub_->publish(wheel_cmd);
  }

  // ---------------- 参数 ----------------
  std::string imu_raw_topic_;
  std::string pose_topic_;
  std::string imu_out_topic_;
  std::string joint_target_topic_;
  std::string wheel_target_topic_;
  std::string joint_states_topic_;
  std::string leg_cmd_topic_;
  std::string wheel_cmd_topic_;

  double imu_rate_{200.0};
  double cmd_rate_{500.0};
  double sensor_timeout_{0.10};

  double wheel_kp_{0.02};
  double wheel_ki_{2.0};
  double wheel_tau_max_{3.0};
  std::string wheel_mode_{"speed"};
  double wheel_torque_gain_{1.0};
  double wheel_start_delay_{1.5};
  bool wheel_arm_on_gate_{true};
  double leg_torque_max_{12.0};
  double command_timeout_{0.25};
  bool flip_roll_pitch_{false};
  std::vector<std::string> leg_joint_order_;
  std::vector<std::string> wheel_joint_order_;

  // ---------------- 缓存 ----------------
  std::array<double, 3> gyro_{0.0, 0.0, 0.0};
  std::array<double, 3> accel_{0.0, 0.0, 0.0};
  std::array<double, 4> quat_{1.0, 0.0, 0.0, 0.0};
  std::string imu_frame_{"imu_site"};
  bool imu_raw_seen_{false};
  bool pose_seen_{false};
  rclcpp::Time last_imu_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_receive_time_{0, 0, RCL_ROS_TIME};
  double pose_time_{0.0};
  double first_pose_time_{0.0};
  bool first_pose_time_set_{false};
  bool gate_seen_{false};
  bool gate_open_{false};

  std::array<double, 4> leg_effort_{0.0, 0.0, 0.0, 0.0};
  std::array<double, 2> wheel_ref_{0.0, 0.0};
  std::array<double, 2> wheel_vel_{0.0, 0.0};
  std::array<double, 2> wheel_i_{0.0, 0.0};
  bool leg_target_seen_{false};
  bool wheel_target_seen_{false};
  bool wheel_meas_seen_{false};
  rclcpp::Time last_ctrl_time_{0, 0, RCL_ROS_TIME};

  // ---------------- ROS 句柄 ----------------
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_raw_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_gate_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;

  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimBridge>());
  rclcpp::shutdown();
  return 0;
}
