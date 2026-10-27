/**
 * @brief 差速轮底盘控制器（差动小车运动学）
 *
 *   /cmd_vel (geometry_msgs/Twist: linear.x = v, angular.z = ω)
 *        │
 *        ▼  ω_L = (v − L/2·ω)/r    ω_R = (v + L/2·ω)/r
 *   左右轮目标转速 [rad/s]
 *        │
 *        ▼
 *   /wheel_target (std_msgs/Float64MultiArray: [左轮, 右轮] rad/s)
 *        │
 *        ▼  与 LQR 的 u 折算速度相加后由 usb_drive 下发（固件轮环是位置-速度串级）
 *   下位机电机
 *
 * 正解（usb_drive 的 odom 用的是同一套公式）：
 *   v = (ω_L + ω_R)·r/2      ω = (ω_R − ω_L)·r/L
 *
 * 符号约定：ω > 0 左转（逆时针）、ω < 0 右转；两个轮速同为正时直线前进。
 * 轮速限幅按左右同比例缩放，保持转向半径不变（见 leg_calc/diff_drive.hpp）。
 *
 * 与其它节点的关系：
 *   * 本节点只负责“行驶”分量；平衡分量仍由 LQR 的 /leg_calc/control 提供，
 *     usb_drive 把两者相加（差速量 + 平衡量 → 左右轮目标速度）。
 *   * 只支持 usb_drive 的 wheel_cmd_mode=velocity；position 模式下差速量被忽略。
 *
 * 调试话题 diff_drive/state：[v, ω, v_L, v_R, ω_L, ω_R, saturated]
 */

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "leg_calc/diff_drive.hpp"

class DiffDriveNode : public rclcpp::Node
{
public:
  DiffDriveNode()
  : Node("leg_calc_diff_drive")
  {
    declareAllParameters();
    applyConfig();

    publish_rate_ = get_parameter("publish_rate").as_double();

    wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(wheel_target_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("diff_drive/state", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {onCmdVel(*msg);});

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return onParameterChange(params);});

    last_cmd_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_)),
      [this]() {step();});

    RCLCPP_INFO(
      get_logger(), "差速底盘就绪：%.0f Hz，订阅 %s，发布 %s",
      publish_rate_, cmd_vel_topic_.c_str(), wheel_target_topic_.c_str());
  }

private:
  // ---------------- 参数 ----------------
  void declareAllParameters()
  {
    const leg_calc::DiffDriveParams params;
    declare_parameter("wheel_separation", params.wheel_separation);
    declare_parameter("wheel_radius", params.wheel_radius);
    declare_parameter("max_wheel_speed", params.max_wheel_speed);
    declare_parameter("max_linear", params.max_linear);
    declare_parameter("max_angular", params.max_angular);

    declare_parameter("cmd_velocity_timeout", 0.3);
    declare_parameter("publish_rate", 50.0);

    declare_parameter("cmd_vel_topic", std::string("cmd_vel"));
    declare_parameter("wheel_target_topic", std::string("wheel_target"));
  }

  void applyConfig(const std::map<std::string, rclcpp::Parameter> & pending = {})
  {
    auto get = [this, &pending](const std::string & name) -> rclcpp::Parameter {
        const auto it = pending.find(name);
        return (it != pending.end()) ? it->second : get_parameter(name);
      };

    leg_calc::DiffDriveParams params;
    params.wheel_separation = get("wheel_separation").as_double();
    params.wheel_radius = get("wheel_radius").as_double();
    params.max_wheel_speed = get("max_wheel_speed").as_double();
    params.max_linear = get("max_linear").as_double();
    params.max_angular = get("max_angular").as_double();
    if (!params.isValid()) {
      throw std::invalid_argument("wheel_separation 与 wheel_radius 必须为正");
    }

    cmd_velocity_timeout_ = get("cmd_velocity_timeout").as_double();
    if (!(cmd_velocity_timeout_ > 0.0)) {
      throw std::invalid_argument("cmd_velocity_timeout 必须为正");
    }

    cmd_vel_topic_ = get("cmd_vel_topic").as_string();
    wheel_target_topic_ = get("wheel_target_topic").as_string();

    params_ = params;
    RCLCPP_INFO(
      get_logger(),
      "底盘参数 L=%.3f m r=%.3f m，限幅 |v|<=%.2f m/s |ω|<=%.2f rad/s |轮速|<=%.1f rad/s，"
      "指令超时 %.2f s",
      params_.wheel_separation, params_.wheel_radius, params_.max_linear,
      params_.max_angular, params_.max_wheel_speed, cmd_velocity_timeout_);
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    static const std::vector<std::string> kTunable = {
      "wheel_separation", "wheel_radius", "max_wheel_speed",
      "max_linear", "max_angular", "cmd_velocity_timeout"};

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
  void onCmdVel(const geometry_msgs::msg::Twist & msg)
  {
    linear_ = msg.linear.x;    // 只用平面运动 v 与 ω，其余分量忽略
    angular_ = msg.angular.z;
    last_cmd_time_ = now();
    cmd_received_ = true;
  }

  // ---------------- 发布循环 ----------------
  void step()
  {
    if (!cmd_received_) {
      return;  // 从未收到指令：不发布，usb_drive 侧超时后自动撤销差速量
    }
    const double age = (now() - last_cmd_time_).seconds();
    if (age > cmd_velocity_timeout_) {
      publishCommand(0.0, 0.0);
      cmd_received_ = false;
      RCLCPP_INFO(get_logger(), "cmd_vel 超时 %.2f s，左右轮目标转速置零", age);
      return;
    }
    publishCommand(linear_, angular_);
  }

  void publishCommand(double linear, double angular)
  {
    const leg_calc::DiffDriveState state = leg_calc::wheelsFromBody(params_, linear, angular);

    std_msgs::msg::Float64MultiArray wheel_msg;
    wheel_msg.data = {state.left_speed, state.right_speed};
    wheel_pub_->publish(wheel_msg);

    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data = {
      state.linear, state.angular, state.left_linear, state.right_linear,
      state.left_speed, state.right_speed, state.saturated ? 1.0 : 0.0};
    debug_pub_->publish(debug_msg);

    if (state.saturated) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "指令被限幅：v=%.2f m/s ω=%.2f rad/s → 轮速 [%.2f, %.2f] rad/s",
        state.linear, state.angular, state.left_speed, state.right_speed);
    }
  }

  // ---------------- 参数 ----------------
  leg_calc::DiffDriveParams params_;
  double cmd_velocity_timeout_{0.3};
  double publish_rate_{50.0};
  std::string cmd_vel_topic_{"cmd_vel"};
  std::string wheel_target_topic_{"wheel_target"};

  // 输入缓存
  double linear_{0.0};
  double angular_{0.0};
  bool cmd_received_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveNode>());
  rclcpp::shutdown();
  return 0;
}
