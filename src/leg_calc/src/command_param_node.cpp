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
 * 上层速度命令有两条入口，优先级相同（谁后写谁生效）：
 *   ① 参数 linear_velocity / yaw_rate（rqt 或 ros2 param set，适合手动调试）
 *   ② 话题 /cmd_vel（geometry_msgs/Twist，适合导航/遥控程序；收到即切 TELEOP）
 *      · 超过 cmd_vel_timeout 没有新消息就把速度归零，避免上层掉线后机器人继续跑
 *      · 只取 linear.x 与 angular.z，其余分量忽略
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

#include <geometry_msgs/msg/twist.hpp>
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
    cmd_vel_topic_ = declare_parameter("cmd_vel_topic", std::string("cmd_vel"), read_only);
    use_cmd_vel_ = declare_parameter("use_cmd_vel", true, read_only);
    cmd_vel_timeout_ = declare_parameter("cmd_vel_timeout", 0.5, read_only);
    max_linear_accel_ = declare_parameter("max_linear_accel", 0.30, read_only);
    max_yaw_accel_ = declare_parameter("max_yaw_accel", 1.0, read_only);
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

    // 与 leg_control 的订阅端保持一致：best_effort + depth 1（只要最新一帧）
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      command_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    if (use_cmd_vel_) {
      cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {onCmdVel(*msg);});
    }
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
    if (use_cmd_vel_) {
      RCLCPP_INFO(
        get_logger(),
        "上层速度命令：订阅 %s（Twist，linear.x / angular.z，收到即切 TELEOP，断流 %.2f s 归零）",
        cmd_vel_topic_.c_str(), cmd_vel_timeout_);
    }
    RCLCPP_INFO(
      get_logger(), "当前 enable=%s（launch 的 enable 参数默认 false）；使能：ros2 param set /command_param enable true",
      enable_ ? "true" : "false");
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
    if (!(cmd_vel_timeout_ > 0.0)) {
      throw std::invalid_argument("cmd_vel_timeout 必须为正");
    }
    if (!(max_linear_accel_ > 0.0) || !(max_yaw_accel_ > 0.0)) {
      throw std::invalid_argument("max_linear_accel / max_yaw_accel 必须为正");
    }
    if (use_cmd_vel_ && cmd_vel_topic_.empty()) {
      throw std::invalid_argument("cmd_vel_topic 不能为空（use_cmd_vel=false 可关闭）");
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

    // 参数一旦被显式设定，就以参数为准，不再受 /cmd_vel 断流超时影响
    for (const auto & param : params) {
      if (param.get_name() == "linear_velocity" || param.get_name() == "yaw_rate") {
        cmd_vel_seen_ = false;
      }
    }

    enable_ = enable;
    teleop_ = teleop;
    estop_ = estop;
    linear_target_ = linear;   // 只改目标，实际发布值在 publishCommand 里按加速度斜坡跟上
    yaw_target_ = yaw;
    com_height_ = height;
    pitch_target_ = pitch;

    RCLCPP_INFO(
      get_logger(), "上层指令更新：mode=%s, estop=%s, v目标=%.3f, yaw目标=%.3f, h=%.3f, pitch=%.3f",
      modeName(), estop_ ? "ON" : "OFF", linear_target_, yaw_target_, com_height_, pitch_target_);
    return result;
  }

  const char * modeName() const
  {
    if (!enable_) {
      return "ZERO";
    }
    return teleop_ ? "TELEOP" : "STAND";
  }

  /// 上层速度命令（geometry_msgs/Twist）：只取 linear.x / angular.z，收到即切 TELEOP
  void onCmdVel(const geometry_msgs::msg::Twist & msg)
  {
    if (!std::isfinite(msg.linear.x) || !std::isfinite(msg.angular.z)) {
      return;
    }
    linear_target_ = std::clamp(msg.linear.x, -max_linear_, max_linear_);
    yaw_target_ = std::clamp(msg.angular.z, -max_yaw_, max_yaw_);
    last_cmd_vel_ = now();
    cmd_vel_seen_ = true;
    if (!teleop_) {
      teleop_ = true;
      RCLCPP_INFO(get_logger(), "收到 %s，切到 TELEOP", cmd_vel_topic_.c_str());
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "cmd_vel：目标 v=%.3f m/s，yaw=%.3f rad/s",
      linear_target_, yaw_target_);
  }

  void publishCommand()
  {
    // 上层速度命令断流：目标归零，避免机器人带着最后一次速度一直跑
    if (cmd_vel_seen_ && (now() - last_cmd_vel_).seconds() > cmd_vel_timeout_) {
      linear_target_ = 0.0;
      yaw_target_ = 0.0;
    }

    // 速度斜坡：力矩型平衡环对"速度目标的阶跃"非常敏感（0.3 m/s 阶跃对应的
    // 减速力矩约 0.35 N·m，会让俯仰角速度瞬间冲过 3 rad/s 触发 0x08），
    // 所以按 max_linear_accel / max_yaw_accel 限速跟随目标。
    const double dt = 1.0 / publish_rate_;
    const double dv = max_linear_accel_ * dt;
    const double dw = max_yaw_accel_ * dt;
    linear_velocity_ += std::clamp(linear_target_ - linear_velocity_, -dv, dv);
    yaw_rate_ += std::clamp(yaw_target_ - yaw_rate_, -dw, dw);

    double linear = linear_velocity_;
    double yaw = yaw_rate_;

    leg_calc::RobotCommand command;
    command.com_height = com_height_;
    command.pitch_target = pitch_target_;
    command.enable = enable_;
    command.estop = estop_;
    if (enable_ && teleop_ && !estop_) {
      command.linear_velocity = linear;
      command.yaw_rate = yaw;
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
  std::string cmd_vel_topic_{"cmd_vel"};
  bool use_cmd_vel_{true};
  double cmd_vel_timeout_{0.5};
  double max_linear_accel_{0.30};
  double max_yaw_accel_{1.0};
  bool cmd_vel_seen_{false};
  rclcpp::Time last_cmd_vel_{0, 0, RCL_ROS_TIME};

  bool enable_{false};
  bool teleop_{false};
  bool estop_{false};
  double linear_target_{0.0};      // 目标值（参数 / cmd_vel 写入）
  double yaw_target_{0.0};
  double linear_velocity_{0.0};    // 斜坡后的实际下发值
  double yaw_rate_{0.0};
  double com_height_{0.18};
  double pitch_target_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
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
