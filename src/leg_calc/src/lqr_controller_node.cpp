/**
 * @brief 高度自适应 LQR 平衡控制器
 *
 *   /imu/data     → 欧拉角 → theta, theta_dot
 *   /odom         → x, x_dot
 *   /joint_states (4 个腿关节) → 质心高度 h → 反馈增益 K(h)
 *   u = -K(h) * (x - x_ref)
 *
 * 状态向量 x = [x, theta, x_dot, theta_dot]，theta 默认取 IMU 的 pitch 轴。
 *
 * 增益来源（参数 gain_mode）：
 *   quadratic   : K_j(h) = c2*h^2 + c1*h + c0，系数由 k_coeffs(4x3) 给出，
 *                 由 `ros2 run leg_calc gain_table` 扫描 h 求 LQR 后拟合得到；
 *                 若 k_coeffs 全零则自动退回 online_lqr。
 *   online_lqr  : 每个新的 h 现场重建模型并求解 Riccati 方程。
 *
 * 所有增益相关参数（k_coeffs / q_diag / r_weight / gain_mode / 腿部几何 /
 * h_min / h_max / u_max / theta_offset ...）都支持运行时 `ros2 param set` 修改，
 * 修改后立即重建增益调度器，无需重启节点。
 *
 * 输出话题：
 *   leg_calc/state      [x, theta, x_dot, theta_dot]
 *   leg_calc/control    [u]
 *   leg_calc/gain       [k_x, k_theta, k_x_dot, k_theta_dot]   —— 便于记录拟合数据
 *   leg_calc/com_height [h]
 *   leg_calc/euler      [roll, pitch, yaw] (geometry_msgs/Vector3Stamped)
 */

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "leg_calc/lqr.hpp"
#include "leg_calc/param_utils.hpp"
#include "leg_calc/quat_utils.hpp"
#include "leg_calc/wheel_leg_model.hpp"

class LqrControllerNode : public rclcpp::Node
{
public:
  LqrControllerNode()
  : Node("leg_calc_lqr_controller")
  {
    declareAllParameters();
    applyConfig();

    control_rate_ = declare_parameter("control_rate", control_rate_);

    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("leg_calc/state", 10);
    control_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("leg_calc/control", 10);
    gain_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("leg_calc/gain", 10);
    height_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("leg_calc/com_height", 10);
    euler_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("leg_calc/euler", 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {onImu(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onOdom(*msg);});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJointStates(*msg);});

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return onParameterChange(params);});

    last_imu_time_ = now();
    last_joint_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_)),
      [this]() {step();});
  }

private:
  // ---------------- 参数 ----------------
  void declareAllParameters()
  {
    leg_calc::declareModelParams(*this);
    leg_calc::declareLqrWeights(*this);
    declare_parameter("u_max", 5.0);

    declare_parameter("theta_axis", std::string("pitch"));
    declare_parameter("theta_offset", 0.0);
    declare_parameter("imu_timeout", 0.2);

    declare_parameter("imu_topic", std::string("imu/data"));
    declare_parameter("odom_topic", std::string("odom"));
    declare_parameter("joint_states_topic", std::string("joint_states"));

    declare_parameter(
      "leg_joint_names",
      std::vector<std::string>{
        "leg_left_hip_joint", "leg_left_knee_joint",
        "leg_right_hip_joint", "leg_right_knee_joint"});

    declare_parameter("gain_mode", std::string("quadratic"));
    declare_parameter("k_coeffs", std::vector<double>(12, 0.0));

    const leg_calc::LegGeometry geometry;
    declare_parameter("thigh", geometry.thigh);
    declare_parameter("shank", geometry.shank);
    declare_parameter("com_offset", geometry.com_offset);
    declare_parameter("h_min", 0.10);
    declare_parameter("h_max", 0.60);

    declare_parameter("use_measured_height", true);
    declare_parameter("height_timeout", 0.5);

    declare_parameter("reference_state", std::vector<double>{0.0, 0.0, 0.0, 0.0});
  }

  /**
   * @brief 读取参数并重建增益调度器
   * @param pending 参数回调中携带的“待提交”参数：on_set_parameters_callback 在参数
   *                真正写入之前触发，此时 get_parameter 仍是旧值，必须优先用这里的新值
   */
  void applyConfig(const std::map<std::string, rclcpp::Parameter> & pending = {})
  {
    auto get = [this, &pending](const std::string & name) -> rclcpp::Parameter {
        const auto it = pending.find(name);
        return (it != pending.end()) ? it->second : get_parameter(name);
      };

    const leg_calc::ModelParams params = leg_calc::readModelParams(*this, &pending);
    nominal_height_ = params.com_length;

    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    leg_calc::readLqrWeights(*this, &Q, &R, &pending);
    u_max_ = get("u_max").as_double();

    theta_axis_ = get("theta_axis").as_string();
    axis_index_ = (theta_axis_ == "roll") ? 0 : 1;
    theta_offset_ = get("theta_offset").as_double();
    imu_timeout_ = get("imu_timeout").as_double();

    imu_topic_ = get("imu_topic").as_string();
    odom_topic_ = get("odom_topic").as_string();
    joint_states_topic_ = get("joint_states_topic").as_string();

    leg_joint_names_ = get("leg_joint_names").as_string_array();
    if (leg_joint_names_.size() != 4) {
      throw std::invalid_argument("leg_joint_names 必须包含 4 个关节名");
    }

    leg_geometry_.thigh = get("thigh").as_double();
    leg_geometry_.shank = get("shank").as_double();
    leg_geometry_.com_offset = get("com_offset").as_double();
    use_measured_height_ = get("use_measured_height").as_bool();
    height_timeout_ = get("height_timeout").as_double();

    const std::vector<double> x_ref = get("reference_state").as_double_array();
    if (x_ref.size() != static_cast<size_t>(leg_calc::kStateDim)) {
      throw std::invalid_argument("reference_state 必须包含 4 个元素");
    }
    for (int i = 0; i < leg_calc::kStateDim; ++i) {
      x_ref_(i) = x_ref[static_cast<size_t>(i)];
    }

    // ---- 增益调度器 ----
    const double h_min = get("h_min").as_double();
    const double h_max = get("h_max").as_double();
    const std::vector<double> k_coeffs = get("k_coeffs").as_double_array();
    if (k_coeffs.size() != 12) {
      throw std::invalid_argument("k_coeffs 必须包含 12 个数（4 个增益 x 3 个系数）");
    }
    Eigen::Matrix<double, 4, 3> coeffs;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        coeffs(i, j) = k_coeffs[static_cast<size_t>(i * 3 + j)];
      }
    }

    gain_schedule_.setModelParams(params);
    gain_schedule_.setMode(
      get("gain_mode").as_string() == "online_lqr" ?
      leg_calc::GainMode::kOnlineLqr : leg_calc::GainMode::kQuadratic);
    gain_schedule_.setWeights(Q, R);
    gain_schedule_.setHeightLimits(h_min, h_max);
    gain_schedule_.setQuadraticCoefficients(coeffs);

    // 启动/变更自检：在名义高度处求一次增益
    const Eigen::Matrix<double, 1, 4> k0 = gain_schedule_.solveLqrAt(nominal_height_);
    if (!gain_schedule_.lastSolveOk()) {
      throw std::runtime_error("LQR 求解失败：请检查 Q/R 与模型参数");
    }
    RCLCPP_INFO(
      get_logger(), "名义高度 h=%.3f m 处 K = [%.4f, %.4f, %.4f, %.4f]",
      nominal_height_, k0(0, 0), k0(0, 1), k0(0, 2), k0(0, 3));

    if (gain_schedule_.hasQuadraticCoefficients()) {
      RCLCPP_INFO(get_logger(), "增益来源：二次多项式 K(h)；系数行序 [k_x, k_theta, k_x_dot, k_theta_dot]");
      for (int i = 0; i < 4; ++i) {
        RCLCPP_INFO(
          get_logger(), "  k[%d](h) = %+.4f*h^2 %+.4f*h %+.4f", i,
          coeffs(i, 0), coeffs(i, 1), coeffs(i, 2));
      }
    } else {
      RCLCPP_INFO(get_logger(), "增益来源：在线 LQR（k_coeffs 全零，按 h 实时求解）");
    }
    RCLCPP_INFO(
      get_logger(),
      "腿部几何 l1=%.3f l2=%.3f com_offset=%.3f, h∈[%.2f, %.2f] m, 实测高度=%s，|u|<=%.2f",
      leg_geometry_.thigh, leg_geometry_.shank, leg_geometry_.com_offset,
      h_min, h_max, use_measured_height_ ? "开" : "关", u_max_);
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    static const std::vector<std::string> kTunable = {
      "cart_mass", "pendulum_mass", "com_length", "com_inertia", "gravity",
      "wheel_radius", "wheel_inertia", "input_mode",
      "q_diag", "r_weight", "u_max",
      "theta_axis", "theta_offset", "imu_timeout",
      "leg_joint_names", "gain_mode", "k_coeffs",
      "thigh", "shank", "com_offset", "h_min", "h_max",
      "use_measured_height", "height_timeout", "reference_state"};

    bool touched = false;
    for (const auto & param : params) {
      if (std::find(kTunable.begin(), kTunable.end(), param.get_name()) != kTunable.end()) {
        touched = true;
        break;
      }
    }
    if (!touched) {
      return result;
    }

    std::map<std::string, rclcpp::Parameter> pending;
    for (const auto & param : params) {
      pending[param.get_name()] = param;
    }

    try {
      applyConfig(pending);
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = e.what();
    }
    return result;
  }

  // ---------------- 输入回调 ----------------
  void onImu(const sensor_msgs::msg::Imu & msg)
  {
    const auto & q = msg.orientation;
    if (msg.orientation_covariance[0] < 0.0 && !warned_no_orientation_) {
      RCLCPP_WARN(get_logger(), "IMU 未提供姿态四元数（orientation_covariance[0] < 0）");
      warned_no_orientation_ = true;
    }

    const Eigen::Vector3d rpy = leg_calc::quaternionToRpy(q.w, q.x, q.y, q.z);
    theta_ = rpy(axis_index_) - theta_offset_;
    theta_dot_ = (axis_index_ == 0) ? msg.angular_velocity.x : msg.angular_velocity.y;

    euler_msg_.header.stamp = now();
    euler_msg_.header.frame_id = msg.header.frame_id;
    euler_msg_.vector.x = rpy(0);
    euler_msg_.vector.y = rpy(1);
    euler_msg_.vector.z = rpy(2);

    last_imu_time_ = now();
    imu_ok_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    x_ = msg.pose.pose.position.x;
    x_dot_ = msg.twist.twist.linear.x;
    odom_ok_ = true;
  }

  /// 四个腿关节角 → 质心高度
  void onJointStates(const sensor_msgs::msg::JointState & msg)
  {
    std::array<double, 4> angles = joint_angles_;
    for (size_t i = 0; i < leg_joint_names_.size(); ++i) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), leg_joint_names_[i]);
      if (it == msg.name.end()) {
        if (!warned_missing_joint_) {
          RCLCPP_WARN(
            get_logger(), "joint_states 中缺少关节 %s，改用名义高度 %.3f m",
            leg_joint_names_[0].c_str(), nominal_height_);
          warned_missing_joint_ = true;
        }
        return;
      }
      const size_t index = static_cast<size_t>(std::distance(msg.name.begin(), it));
      if (index >= msg.position.size()) {
        return;
      }
      angles[i] = msg.position[index];
    }

    joint_angles_ = angles;
    last_joint_time_ = now();
    joint_ok_ = true;
  }

  // ---------------- 控制主循环 ----------------
  void step()
  {
    const double age = (now() - last_imu_time_).seconds();
    if (!imu_ok_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "尚未收到 IMU 数据（主题 %s），输出置零", imu_topic_.c_str());
      publishControl(0.0);
      return;
    }
    if (age > imu_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "IMU 数据超时（%.3f s），输出置零", age);
      publishControl(0.0);
      return;
    }
    if (!odom_ok_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "尚未收到里程计，x / x_dot 按 0 处理");
    }

    // 由关节角估算质心高度，并据此取当前反馈增益
    com_height_ = currentHeight();
    const Eigen::Matrix<double, 1, 4> gain = gain_schedule_.gainAt(com_height_);

    leg_calc::WheelLegModel::State state;
    state << x_, theta_, x_dot_, theta_dot_;
    const double u = -(gain * (state - x_ref_))(0, 0);

    std_msgs::msg::Float64MultiArray state_msg;
    state_msg.data = {state(0), state(1), state(2), state(3)};
    state_pub_->publish(state_msg);

    std_msgs::msg::Float64MultiArray gain_msg;
    gain_msg.data = {gain(0, 0), gain(0, 1), gain(0, 2), gain(0, 3)};
    gain_pub_->publish(gain_msg);

    std_msgs::msg::Float64MultiArray height_msg;
    height_msg.data = {com_height_};
    height_pub_->publish(height_msg);

    euler_pub_->publish(euler_msg_);
    publishControl(u);
  }

  /// 质心高度：优先用实测关节角，缺失或超时则用名义值
  double currentHeight() const
  {
    if (!use_measured_height_ || !joint_ok_) {
      return nominal_height_;
    }
    if ((now() - last_joint_time_).seconds() > height_timeout_) {
      return nominal_height_;
    }
    return leg_geometry_.comHeight(joint_angles_);
  }

  void publishControl(double u)
  {
    const double clamped = std::clamp(u, -u_max_, u_max_);
    if (std::abs(clamped - u) > 1e-9) {
      RCLCPP_DEBUG(get_logger(), "控制量饱和: %.3f -> %.3f", u, clamped);
    }
    std_msgs::msg::Float64MultiArray control_msg;
    control_msg.data = {clamped};
    control_pub_->publish(control_msg);
  }

  leg_calc::GainSchedule gain_schedule_;
  leg_calc::LegGeometry leg_geometry_;
  leg_calc::WheelLegModel::State x_ref_{leg_calc::WheelLegModel::State::Zero()};

  double control_rate_{200.0};
  double u_max_{5.0};
  double imu_timeout_{0.2};
  double height_timeout_{0.5};
  double nominal_height_{0.30};
  double com_height_{0.30};
  double theta_offset_{0.0};
  int axis_index_{1};
  bool use_measured_height_{true};
  std::string theta_axis_{"pitch"};
  std::string imu_topic_{"imu/data"};
  std::string odom_topic_{"odom"};
  std::string joint_states_topic_{"joint_states"};
  std::vector<std::string> leg_joint_names_;

  // 状态量
  double x_{0.0};
  double theta_{0.0};
  double x_dot_{0.0};
  double theta_dot_{0.0};
  std::array<double, 4> joint_angles_{0.0, 0.0, 0.0, 0.0};
  bool imu_ok_{false};
  bool odom_ok_{false};
  bool joint_ok_{false};
  bool warned_no_orientation_{false};
  bool warned_missing_joint_{false};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Vector3Stamped euler_msg_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gain_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr height_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LqrControllerNode>());
  rclcpp::shutdown();
  return 0;
}
