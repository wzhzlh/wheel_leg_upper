#ifndef LEG_CALC__LEG_CONTROL_IO_HPP_
#define LEG_CALC__LEG_CONTROL_IO_HPP_

// 输入与状态估计 / 输出打包（只有“搬数据”，没有控制逻辑）
//
//   SnapshotIO        : 把 /imu/data、/odom、/joint_states、/robot_command
//                       缓存成 StateSnapshot（含各通道年龄）
//   makeJointTarget   : ControlOutput → /joint_target（effort = τ）
//   makeWheelTarget   : ControlOutput → /wheel_target（ω_L, ω_R）
//   makeStateDebug    : ControlOutput → /leg_control/state（调试量）

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <string>
#include <vector>

#include "leg_calc/leg_control_types.hpp"

namespace leg_calc
{

/// 各输入通道的时间戳缓存 → 一致快照
class SnapshotIO
{
public:
  /// 姿态通道；stamp_s 为收到该消息时的秒数
  void onImu(
    const sensor_msgs::msg::Imu & msg, int axis_index, double theta_offset, double stamp_s);

  /// 关节通道：同时拆出 4 个腿关节与 2 个轮关节（轮端角度用于本轮里程计）
  void onJointStates(
    const sensor_msgs::msg::JointState & msg,
    const std::vector<std::string> & leg_joint_names,
    const std::vector<std::string> & wheel_joint_names, double stamp_s);

  void onCommand(const std_msgs::msg::Float64MultiArray & msg, double stamp_s);

  /// 组装快照：age 由 stamp_s 与各通道最近一次时间戳算出；
  /// x / x_dot 由轮端角度、角速度按 r 换算（轮半径的权威来源是控制参数）
  StateSnapshot build(double stamp_s, double wheel_radius) const;

  /// 轮端里程计：x = r·(φL+φR)/2
  double wheelTravel(double wheel_radius) const;
  double wheelSpeed(double wheel_radius) const;
  double wheelYawRate(double wheel_radius, double wheel_separation) const;

  bool commandSeen() const {return command_seen_;}

private:
  double pitch_{0.0};
  double pitch_rate_{0.0};
  double yaw_{0.0};
  double yaw_rate_{0.0};
  std::array<double, 4> joint_position_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> joint_velocity_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 2> wheel_angle_{{0.0, 0.0}};
  std::array<double, 2> wheel_omega_{{0.0, 0.0}};

  bool imu_ok_{false};
  bool joint_ok_{false};
  bool wheel_ok_{false};
  bool command_seen_{false};

  double last_imu_s_{-1e9};
  double last_joint_s_{-1e9};
  double last_wheel_s_{-1e9};   ///< 轮反馈单独记时间：它决定 LQR 的 x / x_dot 能不能用
  double last_command_s_{-1e9};

  RobotCommand command_{};
};

sensor_msgs::msg::JointState makeJointTarget(
  const std::vector<std::string> & leg_joint_names, const StateSnapshot & snapshot,
  const ControlOutput & output, const rclcpp::Time & stamp);

std_msgs::msg::Float64MultiArray makeWheelTarget(const ControlOutput & output);

std_msgs::msg::Float64MultiArray makeStateDebug(
  const StateSnapshot & snapshot, const ControlOutput & output);

}  // namespace leg_calc

#endif  // LEG_CALC__LEG_CONTROL_IO_HPP_
