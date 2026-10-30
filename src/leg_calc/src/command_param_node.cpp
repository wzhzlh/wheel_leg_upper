/**
 * @brief 上层指令参数服务节点（rqt_reconfigure / ros2 param 可动态修改）
 *
 * 节点本身只暴露参数并周期发布 /robot_command，不参与任何控制计算。
 * 保留 leg_control_node 原有的 Float64MultiArray 上层接口：
 *   [v, yaw, com_height, pitch_target, enable, estop]
 *
 * 模式映射：
 *   enable=false                    -> ZERO（失能）
 *   enable=true, teleop=false      -> STAND（站立保持）
 *   enable=true, teleop=true       -> TELEOP（采用速度参数）
 *   estop=true                     -> 急停
 *
 * ROS 2 节点自动提供 get_parameters / set_parameters 等参数服务，无需自定义 srv。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "leg_calc/robot_command.hpp"

class CommandParamNode : public rclcpp::Node
{
public:
  CommandParamNode()
  : Node("command_param")
  {
    const auto read_only = readOnlyDescription();
    command_topic_ = declare_parameter("command_topic", std::string("robot_command"), read_only);
    publish_rate_ = declare_parameter("publish_rate", 50.0, read_only);
    max_linear_ = declare_parameter("max_linear", 1.0, read_only);
    max_yaw_ = declare_parameter("max_yaw", 3.0, read_only);
    h_min_ = declare_parameter("com_height_min", 0.065, read_only);
    h_max_ = declare_parameter("com_height_max", 0.100, read_only);
    max_pitch_target_ = declare_parameter("max_pitch_target", 0.20, read_only);
    validateStaticConfig();

    enable_ = declare_parameter("enable", false, boolDescription("使能站立/遥控"));
    teleop_ = declare_parameter("teleop", false, boolDescription("遥控模式；需同时 enable=true"));
    estop_ = declare_parameter("estop", false, boolDescription("急停"));
    linear_velocity_ = declare_parameter(
      "linear_velocity", 0.0,
      rangeDescription("目标线速度 [m/s]，仅遥控模式生效", -max_linear_, max_linear_, 0.01));
    yaw_rate_ = declare_parameter(
      "yaw_rate", 0.0,
      rangeDescription("目标偏航角速度 [rad/s]，仅遥控模式生效", -max_yaw_, max_yaw_, 0.01));
    // 默认值要与 leg_control_params.yaml 的 com_height_default 一致；
    // home 姿态 foot_y≈0.117 m，质心比髋低约 0.032 m。
    com_height_ = declare_parameter(
      "com_height", 0.085,
      rangeDescription("目标质心高度 [m]", h_min_, h_max_, 0.0));
    pitch_target_ = declare_parameter(
      "pitch_target", 0.0,
      rangeDescription("目标 pitch [rad]", -max_pitch_target_, max_pitch_target_, 0.005));

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return onSetParameters(params);});
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      [this]() {publishCommand();});

    RCLCPP_INFO(
      get_logger(),
      "上层参数指令节点启动：/command_param/set_parameters → %s（%.0f Hz）；"
      "enable/teleop/estop/linear_velocity/yaw_rate/com_height/pitch_target 可用 rqt 动态修改",
      command_topic_.c_str(), publish_rate_);
  }

private:
  static rcl_interfaces::msg::ParameterDescriptor readOnlyDescription()
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "启动配置；请修改 YAML 后重启节点";
    descriptor.read_only = true;
    return descriptor;
  }

  static rcl_interfaces::msg::ParameterDescriptor boolDescription(const std::string & text)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = text;
    return descriptor;
  }

  static rcl_interfaces::msg::ParameterDescriptor rangeDescription(
    const std::string & text, double min, double max, double step)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = text;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = min;
    range.to_value = max;
    range.step = step;
    descriptor.floating_point_range = {range};
    return descriptor;
  }

  void validateStaticConfig() const
  {
    if (command_topic_.empty() || !(publish_rate_ > 0.0)) {
      throw std::invalid_argument("command_topic 不能为空，publish_rate 必须为正");
    }
    if (!(max_linear_ > 0.0) || !(max_yaw_ > 0.0) || !(max_pitch_target_ > 0.0)) {
      throw std::invalid_argument("max_linear / max_yaw / max_pitch_target 必须为正");
    }
    if (!(h_min_ < h_max_)) {
      throw std::invalid_argument("com_height_min 必须小于 com_height_max");
    }
  }

  static bool inRange(double value, double min, double max)
  {
    return std::isfinite(value) && value >= min && value <= max;
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool enable = enable_;
    bool teleop = teleop_;
    bool estop = estop_;
    double linear = linear_velocity_;
    double yaw = yaw_rate_;
    double height = com_height_;
    double pitch = pitch_target_;

    for (const auto & param : params) {
      const std::string & name = param.get_name();
      if (name == "enable") {
        enable = param.as_bool();
      } else if (name == "teleop") {
        teleop = param.as_bool();
      } else if (name == "estop") {
        estop = param.as_bool();
      } else if (name == "linear_velocity") {
        linear = param.as_double();
      } else if (name == "yaw_rate") {
        yaw = param.as_double();
      } else if (name == "com_height") {
        height = param.as_double();
      } else if (name == "pitch_target") {
        pitch = param.as_double();
      }
    }

    if (!inRange(linear, -max_linear_, max_linear_)) {
      result.successful = false;
      result.reason = "linear_velocity 超出 max_linear 限幅";
    } else if (!inRange(yaw, -max_yaw_, max_yaw_)) {
      result.successful = false;
      result.reason = "yaw_rate 超出 max_yaw 限幅";
    } else if (!inRange(height, h_min_, h_max_)) {
      result.successful = false;
      result.reason = "com_height 超出配置范围";
    } else if (!inRange(pitch, -max_pitch_target_, max_pitch_target_)) {
      result.successful = false;
      result.reason = "pitch_target 超出配置范围";
    }
    if (!result.successful) {
      return result;
    }

    enable_ = enable;
    teleop_ = teleop;
    estop_ = estop;
    linear_velocity_ = linear;
    yaw_rate_ = yaw;
    com_height_ = height;
    pitch_target_ = pitch;

    RCLCPP_INFO(
      get_logger(), "上层指令更新：mode=%s, estop=%s, v=%.3f, yaw=%.3f, h=%.3f, pitch=%.3f",
      modeName(), estop_ ? "ON" : "OFF", linear_velocity_, yaw_rate_, com_height_, pitch_target_);
    return result;
  }

  const char * modeName() const
  {
    if (!enable_) {
      return "ZERO";
    }
    return teleop_ ? "TELEOP" : "STAND";
  }

  void publishCommand()
  {
    leg_calc::RobotCommand command;
    command.com_height = com_height_;
    command.pitch_target = pitch_target_;
    command.enable = enable_;
    command.estop = estop_;
    if (enable_ && teleop_ && !estop_) {
      command.linear_velocity = linear_velocity_;
      command.yaw_rate = yaw_rate_;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = command.toArray();
    pub_->publish(msg);
  }

  std::string command_topic_;
  double publish_rate_{50.0};
  double max_linear_{1.0};
  double max_yaw_{3.0};
  double h_min_{0.12};
  double h_max_{0.24};
  double max_pitch_target_{0.20};

  bool enable_{false};
  bool teleop_{false};
  bool estop_{false};
  double linear_velocity_{0.0};
  double yaw_rate_{0.0};
  double com_height_{0.18};
  double pitch_target_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandParamNode>());
  rclcpp::shutdown();
  return 0;
}
