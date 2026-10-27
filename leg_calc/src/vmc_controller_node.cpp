/**
 * @brief 足端虚拟模型控制（VMC / 虚拟弹簧-阻尼）
 *
 *   /joint_states  (4 个腿关节角 + 角速度)
 *        │ 正运动学 T(q) / 正雅可比 J(q)
 *        ▼
 *   足端位置 x、速度 ẋ = J·q̇
 *        │ F = Kp·(x_d − x) + Kd·(ẋ_d − ẋ)      （逐轴弹簧 + 阻尼）
 *        ▼
 *   足端期望力 F
 *        │ τ = Jᵀ·F                             （虚功原理，VMC 核心）
 *        ▼
 *   /joint_target (sensor_msgs/JointState, effort = 关节力矩)
 *
 * 五杆几何、坐标系与公式推导见 leg_calc/foot_vmc.hpp。
 *
 * 关节顺序：每腿 [α, β] = [近端髋, 远端髋]，对应 leg_joint_names 的
 * [左近端, 左远端, 右近端, 右远端]；两条腿共用同一套几何与增益。
 *
 * 安全策略：
 *   * 从未收到 joint_states → 不发布 /joint_target（下位机保持“关节失力”）
 *   * joint_states 超时 → 力矩全零，位置/速度回传实测值，避免误保持到 0 位
 *   * 某腿正运动学无解 / 雅可比奇异 → 该腿力矩为零，另一条腿照常输出
 *   * 关节力矩按 torque_max 逐关节限幅，足端力按 force_max 逐轴限幅
 *
 * 调试话题 vmc/state：每腿 8 个数
 *   [x, y, ẋ, ẏ, Fx, Fy, τ_α, τ_β]（左腿在前）
 */

#include <rclcpp/rclcpp.hpp>

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

#include "leg_calc/foot_vmc.hpp"

class VmcControllerNode : public rclcpp::Node
{
public:
  VmcControllerNode()
  : Node("leg_calc_vmc_controller")
  {
    declareAllParameters();
    applyConfig();

    control_rate_ = get_parameter("control_rate").as_double();

    joint_target_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_target_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("vmc/state", 10);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJointStates(*msg);});

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return onParameterChange(params);});

    last_joint_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_)),
      [this]() {step();});

    RCLCPP_INFO(
      get_logger(), "VMC 就绪：控制频率 %.0f Hz，发布 %s，订阅 %s",
      control_rate_, joint_target_topic_.c_str(), joint_states_topic_.c_str());
  }

private:
  // ---------------- 参数 ----------------
  void declareAllParameters()
  {
    const leg_calc::FiveBarGeometry geometry;
    declare_parameter("l1", geometry.l1);
    declare_parameter("l2", geometry.l2);
    declare_parameter("l3", geometry.l3);
    declare_parameter("l4", geometry.l4);
    declare_parameter("l5", geometry.l5);
    declare_parameter("foot_branch", geometry.foot_branch);
    declare_parameter("front_branch", geometry.front_branch);
    declare_parameter("rear_branch", geometry.rear_branch);

    const leg_calc::SpringDamperParams vmc;
    declare_parameter("kp_x", vmc.kp(0));
    declare_parameter("kp_y", vmc.kp(1));
    declare_parameter("kd_x", vmc.kd(0));
    declare_parameter("kd_y", vmc.kd(1));
    declare_parameter("force_max_x", vmc.force_max(0));
    declare_parameter("force_max_y", vmc.force_max(1));

    declare_parameter("foot_target_x", 0.05);
    declare_parameter("foot_target_y", 0.30);
    declare_parameter("foot_vel_target_x", 0.0);
    declare_parameter("foot_vel_target_y", 0.0);

    declare_parameter("torque_max", 8.0);
    declare_parameter("joint_torque_sign", std::vector<double>{1.0, 1.0, 1.0, 1.0});

    declare_parameter("control_rate", 200.0);
    declare_parameter("joint_state_timeout", 0.2);

    declare_parameter("joint_states_topic", std::string("joint_states"));
    declare_parameter("joint_target_topic", std::string("joint_target"));
    declare_parameter(
      "leg_joint_names",
      std::vector<std::string>{
      "leg_left_hip_joint", "leg_left_knee_joint",
      "leg_right_hip_joint", "leg_right_knee_joint"});
  }

  /**
   * @brief 读取参数并校验
   * @param pending 参数回调中携带的“待提交”参数（回调早于参数写入，必须优先使用）
   */
  void applyConfig(const std::map<std::string, rclcpp::Parameter> & pending = {})
  {
    auto get = [this, &pending](const std::string & name) -> rclcpp::Parameter {
        const auto it = pending.find(name);
        return (it != pending.end()) ? it->second : get_parameter(name);
      };

    leg_calc::FiveBarGeometry geometry;
    geometry.l1 = get("l1").as_double();
    geometry.l2 = get("l2").as_double();
    geometry.l3 = get("l3").as_double();
    geometry.l4 = get("l4").as_double();
    geometry.l5 = get("l5").as_double();
    geometry.foot_branch = static_cast<int>(get("foot_branch").as_int());
    geometry.front_branch = static_cast<int>(get("front_branch").as_int());
    geometry.rear_branch = static_cast<int>(get("rear_branch").as_int());
    if (!geometry.isValid()) {
      throw std::invalid_argument(
              "五杆几何非法：l1..l5 必须为正，且 α=β=90° 时 |AC| 须在 [|l2-l3|, l2+l3] 内");
    }

    leg_calc::SpringDamperParams vmc;
    vmc.kp = Eigen::Vector2d(get("kp_x").as_double(), get("kp_y").as_double());
    vmc.kd = Eigen::Vector2d(get("kd_x").as_double(), get("kd_y").as_double());
    vmc.force_max =
      Eigen::Vector2d(get("force_max_x").as_double(), get("force_max_y").as_double());
    if ((vmc.kp.array() < 0.0).any() || (vmc.kd.array() < 0.0).any()) {
      throw std::invalid_argument("kp / kd 不能为负");
    }

    const double torque_max = get("torque_max").as_double();
    if (!(torque_max > 0.0)) {
      throw std::invalid_argument("torque_max 必须为正");
    }
    const std::vector<double> sign = get("joint_torque_sign").as_double_array();
    if (sign.size() != 4) {
      throw std::invalid_argument("joint_torque_sign 必须包含 4 个数（±1）");
    }

    std::vector<std::string> names = get("leg_joint_names").as_string_array();
    if (names.size() != 4) {
      throw std::invalid_argument("leg_joint_names 必须包含 4 个关节名");
    }

    geometry_ = geometry;
    vmc_ = vmc;
    torque_max_ = torque_max;
    for (size_t i = 0; i < 4; ++i) {
      torque_sign_[i] = sign[i];
    }
    foot_target_ =
      Eigen::Vector2d(get("foot_target_x").as_double(), get("foot_target_y").as_double());
    foot_velocity_target_ = Eigen::Vector2d(
      get("foot_vel_target_x").as_double(), get("foot_vel_target_y").as_double());
    joint_state_timeout_ = get("joint_state_timeout").as_double();

    leg_joint_names_ = names;
    joint_states_topic_ = get("joint_states_topic").as_string();
    joint_target_topic_ = get("joint_target_topic").as_string();

    // 自检：α = β = 90° 时的名义足端位置，便于与实机对照
    constexpr double kHalfPi = 1.5707963267948966;
    Eigen::Vector2d neutral = Eigen::Vector2d::Zero();
    const bool neutral_ok = geometry_.footPosition(
      Eigen::Vector2d(kHalfPi, kHalfPi), &neutral);
    RCLCPP_INFO(
      get_logger(),
      "五杆 l1=%.3f l2=%.3f l3=%.3f l4=%.3f l5=%.3f（%.3f, %.3f 为 α=β=90° 足端）",
      geometry_.l1, geometry_.l2, geometry_.l3, geometry_.l4, geometry_.l5,
      neutral.x(), neutral.y());
    if (!neutral_ok) {
      RCLCPP_WARN(get_logger(), "α=β=90° 位形无解，请检查连杆长度");
    }
    RCLCPP_INFO(
      get_logger(), "目标足端 [%.3f, %.3f] m，目标速度 [%.3f, %.3f] m/s，"
      "Kp=[%.1f, %.1f] N/m，Kd=[%.1f, %.1f] N·s/m",
      foot_target_.x(), foot_target_.y(), foot_velocity_target_.x(), foot_velocity_target_.y(),
      vmc_.kp(0), vmc_.kp(1), vmc_.kd(0), vmc_.kd(1));
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    static const std::vector<std::string> kTunable = {
      "l1", "l2", "l3", "l4", "l5", "foot_branch", "front_branch", "rear_branch",
      "kp_x", "kp_y", "kd_x", "kd_y", "force_max_x", "force_max_y",
      "foot_target_x", "foot_target_y", "foot_vel_target_x", "foot_vel_target_y",
      "torque_max", "joint_torque_sign", "joint_state_timeout", "leg_joint_names"};

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

  // ---------------- 输入 ----------------
  void onJointStates(const sensor_msgs::msg::JointState & msg)
  {
    std::array<double, 4> position{};
    std::array<double, 4> velocity{};
    for (size_t i = 0; i < leg_joint_names_.size(); ++i) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), leg_joint_names_[i]);
      if (it == msg.name.end()) {
        if (!warned_missing_joint_) {
          RCLCPP_WARN(
            get_logger(), "joint_states 缺少关节 %s，力矩输出置零",
            leg_joint_names_[i].c_str());
          warned_missing_joint_ = true;
        }
        return;
      }
      const size_t index = static_cast<size_t>(std::distance(msg.name.begin(), it));
      if (index >= msg.position.size()) {
        return;
      }
      position[i] = msg.position[index];
      velocity[i] = (index < msg.velocity.size()) ? msg.velocity[index] : 0.0;
    }

    joint_position_ = position;
    joint_velocity_ = velocity;
    last_joint_time_ = now();
    joint_ok_ = true;
  }

  // ---------------- 控制主循环 ----------------
  void step()
  {
    std::array<double, 4> torque{};
    std::array<double, 16> debug{};

    const double age = (now() - last_joint_time_).seconds();
    if (!joint_ok_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "尚未收到 joint_states（主题 %s），力矩输出置零", joint_states_topic_.c_str());
      publish(torque, debug);
      return;
    }
    if (age > joint_state_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "joint_states 超时 %.3f s，力矩输出置零", age);
      publish(torque, debug);
      return;
    }

    bool any_ok = false;
    for (int leg = 0; leg < 2; ++leg) {
      const size_t i0 = static_cast<size_t>(2 * leg);
      const Eigen::Vector2d q(joint_position_[i0], joint_position_[i0 + 1]);
      const Eigen::Vector2d q_dot(joint_velocity_[i0], joint_velocity_[i0 + 1]);

      leg_calc::VmcLegState state;
      if (!leg_calc::computeVmcLeg(
          geometry_, vmc_, q, q_dot, foot_target_, foot_velocity_target_, &state))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "%s 腿五杆无解（q=[%.2f, %.2f] rad），该腿力矩置零；请检查关节零位与转向",
          leg == 0 ? "左" : "右", q(0), q(1));
        continue;
      }
      any_ok = true;

      for (size_t k = 0; k < 2; ++k) {
        const size_t joint = i0 + k;
        double tau = state.joint_torque(static_cast<Eigen::Index>(k));
        tau = std::clamp(tau * torque_sign_[joint], -torque_max_, torque_max_);
        torque[joint] = tau;
      }

      const size_t base = static_cast<size_t>(8 * leg);
      debug[base + 0] = state.foot_position.x();
      debug[base + 1] = state.foot_position.y();
      debug[base + 2] = state.foot_velocity.x();
      debug[base + 3] = state.foot_velocity.y();
      debug[base + 4] = state.foot_force.x();
      debug[base + 5] = state.foot_force.y();
      debug[base + 6] = torque[i0];
      debug[base + 7] = torque[i0 + 1];
    }

    if (!any_ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "两条腿均无有效解，力矩输出置零");
    }
    publish(torque, debug);
  }

  void publish(const std::array<double, 4> & torque, const std::array<double, 16> & debug)
  {
    // 还没读到过关节角时不发 joint_target：让下位机的看门狗把关节置为失力，
    // 而不是把 position=0 当成目标角（usb_drive 的 joint_kp>0 时会撞零位）
    if (joint_ok_) {
      // 位置/速度回传实测值：usb_drive 的 joint_kp 为 0 时它们不参与电机指令
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = now();
      msg.name = leg_joint_names_;
      msg.position.assign(joint_position_.begin(), joint_position_.end());
      msg.velocity.assign(joint_velocity_.begin(), joint_velocity_.end());
      msg.effort.assign(torque.begin(), torque.end());
      joint_target_pub_->publish(msg);
    }

    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data.assign(debug.begin(), debug.end());
    debug_pub_->publish(debug_msg);
  }

  // ---------------- 参数 ----------------
  leg_calc::FiveBarGeometry geometry_;
  leg_calc::SpringDamperParams vmc_;
  Eigen::Vector2d foot_target_{0.05, 0.30};
  Eigen::Vector2d foot_velocity_target_{Eigen::Vector2d::Zero()};
  std::array<double, 4> torque_sign_{{1.0, 1.0, 1.0, 1.0}};
  double torque_max_{8.0};

  double control_rate_{200.0};
  double joint_state_timeout_{0.2};
  std::vector<std::string> leg_joint_names_;
  std::string joint_states_topic_{"joint_states"};
  std::string joint_target_topic_{"joint_target"};

  // 输入缓存
  std::array<double, 4> joint_position_{};
  std::array<double, 4> joint_velocity_{};
  bool joint_ok_{false};
  bool warned_missing_joint_{false};
  rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VmcControllerNode>());
  rclcpp::shutdown();
  return 0;
}
