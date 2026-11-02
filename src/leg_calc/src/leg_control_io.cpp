#include "leg_calc/leg_control_io.hpp"

#include <algorithm>
#include <cmath>

#include "leg_calc/quat_utils.hpp"

namespace leg_calc
{

void SnapshotIO::onImu(
  const sensor_msgs::msg::Imu & msg, int axis_index, double theta_offset, double stamp_s)
{
  const auto & q = msg.orientation;
  const double norm2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (!std::isfinite(norm2) || norm2 < 1e-12) {
    imu_ok_ = false;
    return;
  }
  const Eigen::Vector3d rpy = quaternionToRpy(q.w, q.x, q.y, q.z);
  const int axis = std::clamp(axis_index, 0, 2);
  const double pitch = rpy(axis) - theta_offset;
  const double pitch_rate = (axis == 0) ? msg.angular_velocity.x : msg.angular_velocity.y;
  const double yaw = rpy(2);         // 偏航角，供 2D 里程计朝向
  const double yaw_rate = msg.angular_velocity.z;
  // 非有限值（四元数未初始化 / 除零 / NaN）一律当“本帧无效”：
  // 保留旧值但把 imu_ok 清掉，让年龄继续变老，由状态机超时判定
  if (!std::isfinite(pitch) || !std::isfinite(pitch_rate) ||
    !std::isfinite(yaw) || !std::isfinite(yaw_rate))
  {
    imu_ok_ = false;
    return;
  }
  pitch_ = pitch;
  pitch_rate_ = pitch_rate;
  yaw_ = yaw;
  yaw_rate_ = yaw_rate;
  last_imu_s_ = stamp_s;
  imu_ok_ = true;
}

void SnapshotIO::onJointStates(
  const sensor_msgs::msg::JointState & msg,
  const std::vector<std::string> & leg_joint_names,
  const std::vector<std::string> & wheel_joint_names, double stamp_s)
{
  // 同一个 /joint_states 上可能还有别的机器人（多栈共用一个 ROS domain 时很常见）。
  // 完全不含本机关节名的消息要整条忽略：不能让它把 joint_ok_/wheel_ok_ 打 false，
  // 否则每来一条别人的消息就掉一次故障位（实测表现为 0x22 ↔ 0x00 每秒抖、
  // 自检周期永远攒不满，状态停在 INIT）。
  const bool mentions_leg = std::any_of(
    leg_joint_names.begin(), leg_joint_names.end(),
    [&msg](const std::string & name) {
      return std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end();
    });
  const bool mentions_wheel = std::any_of(
    wheel_joint_names.begin(), wheel_joint_names.end(),
    [&msg](const std::string & name) {
      return std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end();
    });
  if (!mentions_leg && !mentions_wheel) {
    return;
  }

  bool all_leg = true;
  for (size_t i = 0; i < leg_joint_names.size() && i < 4; ++i) {
    const auto it = std::find(msg.name.begin(), msg.name.end(), leg_joint_names[i]);
    if (it == msg.name.end()) {
      all_leg = false;
      break;
    }
    const size_t index = static_cast<size_t>(std::distance(msg.name.begin(), it));
    if (index >= msg.position.size() || index >= msg.velocity.size() ||
      !std::isfinite(msg.position[index]) || !std::isfinite(msg.velocity[index]))
    {
      all_leg = false;
      break;
    }
    joint_position_[i] = msg.position[index];
    joint_velocity_[i] = msg.velocity[index];
  }

  bool all_wheel = true;
  for (size_t i = 0; i < wheel_joint_names.size() && i < 2; ++i) {
    const auto it = std::find(msg.name.begin(), msg.name.end(), wheel_joint_names[i]);
    if (it == msg.name.end()) {
      all_wheel = false;
      continue;
    }
    const size_t index = static_cast<size_t>(std::distance(msg.name.begin(), it));
    if (index >= msg.velocity.size() || index >= msg.position.size()) {
      all_wheel = false;
      continue;
    }
    const double angle = msg.position[index];
    const double omega = msg.velocity[index];
    if (!std::isfinite(angle) || !std::isfinite(omega)) {
      // 轮端累加里程计最怕 NaN：一旦写进来，x/x_dot 就永久是 NaN
      all_wheel = false;
      continue;
    }
    wheel_angle_[i] = angle;
    wheel_omega_[i] = omega;
  }
  // 只用“提到本机关节”的那部分结果更新有效位，另一部分交给超时判定
  if (mentions_wheel) {
    wheel_ok_ = all_wheel;
    if (all_wheel) {
      last_wheel_s_ = stamp_s;
    }
  }

  if (mentions_leg) {
    joint_ok_ = all_leg;
    if (all_leg) {
      last_joint_s_ = stamp_s;
    }
  }
}

void SnapshotIO::onCommand(const std_msgs::msg::Float64MultiArray & msg, double stamp_s)
{
  if (msg.data.size() < static_cast<size_t>(kCmdCount)) {
    return;
  }
  if (!std::all_of(
      msg.data.begin(), msg.data.begin() + kCmdCount,
      [](double value) {return std::isfinite(value);}))
  {
    return;
  }
  command_ = RobotCommand::fromArray(msg.data);
  last_command_s_ = stamp_s;
  command_seen_ = true;
}

StateSnapshot SnapshotIO::build(double stamp_s, double wheel_radius) const
{
  StateSnapshot snapshot;
  snapshot.imu_ok = imu_ok_;
  snapshot.imu_age = stamp_s - last_imu_s_;
  snapshot.joint_ok = joint_ok_;
  snapshot.joint_age = stamp_s - last_joint_s_;
  snapshot.wheel_ok = wheel_ok_;
  snapshot.wheel_age = stamp_s - last_wheel_s_;

  snapshot.pitch = pitch_;
  snapshot.pitch_rate = pitch_rate_;
  snapshot.yaw = yaw_;
  snapshot.yaw_rate = yaw_rate_;
  // 里程计由轮端角度/角速度换算，轮半径只从控制参数取一次
  snapshot.x = wheelTravel(wheel_radius);
  snapshot.x_dot = wheelSpeed(wheel_radius);

  snapshot.joint_position = joint_position_;
  snapshot.joint_velocity = joint_velocity_;
  snapshot.wheel_omega = wheel_omega_;

  snapshot.command_seen = command_seen_;
  snapshot.command_age = stamp_s - last_command_s_;
  snapshot.command = command_;
  return snapshot;
}

double SnapshotIO::wheelTravel(double wheel_radius) const
{
  return 0.5 * (wheel_angle_[0] + wheel_angle_[1]) * wheel_radius;
}

double SnapshotIO::wheelSpeed(double wheel_radius) const
{
  return 0.5 * (wheel_omega_[0] + wheel_omega_[1]) * wheel_radius;
}

sensor_msgs::msg::JointState makeJointTarget(
  const std::vector<std::string> & leg_joint_names, const StateSnapshot & snapshot,
  const ControlOutput & output, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name = leg_joint_names;
  // position/velocity 是“目标”（下位机运控帧的伺服参考），不是回传值：
  // 目标角由足端目标 IK 得到；usb_drive 的 joint_kp/joint_kd = 0 时这两个字段被忽略，
  // 行为与“纯力矩”完全一致；把 kp/kd 调大就自动变成 MIT 模式（PD + 力矩前馈）。
  if (output.joint_target_valid) {
    msg.position.assign(
      output.joint_position_target.begin(), output.joint_position_target.end());
    msg.velocity.assign(
      output.joint_velocity_target.begin(), output.joint_velocity_target.end());
  } else {
    // IK 失败：退回实测值（配合 kp=0 无影响）
    msg.position.assign(snapshot.joint_position.begin(), snapshot.joint_position.end());
    msg.velocity.assign(snapshot.joint_velocity.begin(), snapshot.joint_velocity.end());
  }
  msg.effort.assign(output.joint_torque.begin(), output.joint_torque.end());
  if (!output.joints_enabled) {
    // 失力：力矩清零，位置/速度回退成实测值（usb_drive 侧 kp/kd=0 即“自由状态”）
    std::fill(msg.effort.begin(), msg.effort.end(), 0.0);
    msg.position.assign(snapshot.joint_position.begin(), snapshot.joint_position.end());
    msg.velocity.assign(snapshot.joint_velocity.begin(), snapshot.joint_velocity.end());
  }
  return msg;
}

std_msgs::msg::Float64MultiArray makeWheelTarget(const ControlOutput & output)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {output.wheel_omega_ref[0], output.wheel_omega_ref[1]};
  return msg;
}

std_msgs::msg::Float64MultiArray makeWheelTorque(
  const ControlOutput & output, double force_to_torque)
{
  // 力矩通道：[τ_左, τ_右] = 平衡 u + 速度误差 P ± 偏航误差 P，控制核心里已限幅。
  // 左右不再同值：转向就是靠这两轮的力矩差（下位机已不做闭环，/wheel_target 不再生效）。
  // 非平衡态（未使能 / FAULT）时 wheel_torque_ref 是阻尼制动项，丢掉等于轮子自由滚，
  // 所以这里不再按 balance_enabled 清零。
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    output.wheel_torque_ref[0] * force_to_torque,
    output.wheel_torque_ref[1] * force_to_torque};
  return msg;
}

std_msgs::msg::Float64MultiArray makeStateDebug(
  const StateSnapshot & snapshot, const ControlOutput & output)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    static_cast<double>(output.state), static_cast<double>(output.faults),
    snapshot.pitch, snapshot.pitch_rate, snapshot.x, snapshot.x_dot,
    output.u, output.com_height, output.com_height_cmd,
    output.joint_torque[0], output.joint_torque[1],
    output.joint_torque[2], output.joint_torque[3],
    output.wheel_omega_ref[0], output.wheel_omega_ref[1]};
  return msg;
}

}  // namespace leg_calc
