#include "leg_calc/leg_control_core.hpp"

#include <algorithm>
#include <cmath>

namespace leg_calc
{

LegController::LegController(const ControllerConfig & config)
{
  setConfig(config);
}

void LegController::setConfig(const ControllerConfig & config)
{
  const bool first_time = !initialized_;
  config_ = config;

  gains_.setModelParams(config_.model);
  gains_.setMode(config_.gain_mode);
  gains_.setWeights(config_.Q, config_.R);
  gains_.setHeightLimits(config_.h_min, config_.h_max);
  gains_.setQuadraticCoefficients(config_.k_coeffs);
  safety_.setLimits(config_.safety);

  // 首次直接用默认质心高度，之后只把已有目标限幅到新区间，避免参数变化导致跳变
  const double requested =
    (com_height_target_ > 0.0) ? com_height_target_ : config_.com_height_default;
  com_height_target_ = std::clamp(requested, config_.com_height_min, config_.com_height_max);
  if (first_time) {
    com_height_cmd_ = com_height_target_;
  } else {
    com_height_cmd_ = std::clamp(
      com_height_cmd_, config_.com_height_min, config_.com_height_max);
  }
  initialized_ = true;
}

void LegController::reset()
{
  safety_.reset();
  balance_omega_state_ = {{0.0, 0.0}};
  com_height_cmd_ = com_height_target_;
  com_height_ = config_.model.com_length;
}

ControlOutput LegController::step(const StateSnapshot & snapshot, double dt)
{
  com_height_ = computeComHeight(snapshot);
  const GainSchedule::Gain gain = gains_.gainAt(com_height_);

  SafetyInputs inputs;
  inputs.imu_ok = snapshot.imu_ok;
  inputs.imu_age = snapshot.imu_age;
  inputs.joint_ok = snapshot.joint_ok;
  inputs.joint_age = snapshot.joint_age;
  inputs.wheel_ok = snapshot.wheel_ok;
  inputs.wheel_age = snapshot.wheel_age;
  inputs.lqr_ok = gains_.lastSolveOk() && gain.allFinite();
  inputs.command_seen = snapshot.command_seen;
  inputs.command_age = snapshot.command_age;
  inputs.pitch = snapshot.pitch;
  inputs.pitch_rate = snapshot.pitch_rate;
  inputs.estop = snapshot.command.estop;
  inputs.enable_request = snapshot.command.enable && !snapshot.command.estop;

  const SafetyStatus status = safety_.update(inputs);

  ControlOutput out;
  out.state = status.state;
  out.faults = status.faults;
  out.balance_enabled = status.balance_enabled;
  out.joints_enabled = status.joints_enabled;
  out.command_fresh = status.command_fresh;

  if (status.balance_enabled) {
    updateComHeightTarget(snapshot.command, dt);
    out.u = computeBalanceTorque(snapshot, gain);
    out.joint_torque = computeJointTorques(snapshot);
    out.wheel_omega_ref = computeWheelSpeeds(status, snapshot, out.u, dt);
  } else {
    // 未使能 / 故障：关节失力、轮子位置保持（ω_ref 全零），并清掉平衡分量状态
    balance_omega_state_ = {{0.0, 0.0}};
  }

  out.com_height = com_height_;
  out.com_height_cmd = com_height_cmd_;
  return out;
}

void LegController::updateComHeightTarget(const RobotCommand & command, double dt)
{
  double target = command.com_height;
  if (target <= 0.0) {
    target = config_.com_height_default;
  }
  com_height_target_ = std::clamp(
    target, config_.com_height_min, config_.com_height_max);

  const double step = config_.com_height_rate * std::max(0.0, dt);
  com_height_cmd_ += std::clamp(com_height_target_ - com_height_cmd_, -step, step);
}

Eigen::Vector2d LegController::legJointAngles(const StateSnapshot & snapshot, int leg) const
{
  const size_t i0 = static_cast<size_t>(2 * leg);
  return Eigen::Vector2d(
    config_.joint_sign[i0] * snapshot.joint_position[i0] + config_.joint_offset[i0],
    config_.joint_sign[i0 + 1] * snapshot.joint_position[i0 + 1] + config_.joint_offset[i0 + 1]);
}

double LegController::computeComHeight(const StateSnapshot & snapshot) const
{
  if (!config_.use_measured_height || !snapshot.joint_ok ||
    snapshot.joint_age > config_.height_timeout)
  {
    return config_.model.com_length;
  }

  // 五杆闭链：正运动学求左右腿足端位置，质心高度 = 足端 y + 髋→质心偏置。
  // 足端 y 是“轮轴到髋关节的竖直距离”，质心在髋上方 com_offset 处。
  double y_sum = 0.0;
  int valid_legs = 0;
  for (int leg = 0; leg < 2; ++leg) {
    const Eigen::Vector2d q = legJointAngles(snapshot, leg);
    Eigen::Vector2d foot;
    if (config_.fivebar.footPosition(q, &foot)) {
      y_sum += foot.y();
      ++valid_legs;
    }
  }
  if (valid_legs == 0) {
    return config_.model.com_length;
  }
  return y_sum / static_cast<double>(valid_legs) + config_.leg_geometry.com_offset;
}

double LegController::computeBalanceTorque(
  const StateSnapshot & snapshot, const GainSchedule::Gain & gain) const
{
  WheelLegModel::State state;
  state << snapshot.x, snapshot.pitch, snapshot.x_dot, snapshot.pitch_rate;
  WheelLegModel::State ref = WheelLegModel::State::Zero();
  ref(kTheta) = snapshot.command.pitch_target;

  const double u = -(gain * (state - ref))(0, 0);
  return std::clamp(u, -config_.u_max, config_.u_max);
}

std::array<double, 4> LegController::computeJointTorques(const StateSnapshot & snapshot) const
{
  std::array<double, 4> torque{{0.0, 0.0, 0.0, 0.0}};
  // 足端 y 目标 = 目标质心高度 − 髋→质心偏置（得到髋到轮轴的竖直距离）
  const double foot_y_target = com_height_cmd_ - config_.leg_geometry.com_offset;
  const Eigen::Vector2d foot_target(config_.foot_target_x, foot_y_target);

  for (int leg = 0; leg < 2; ++leg) {
    const size_t i0 = static_cast<size_t>(2 * leg);
    const Eigen::Vector2d q = legJointAngles(snapshot, leg);
    const Eigen::Vector2d q_dot(
      config_.joint_sign[i0] * snapshot.joint_velocity[i0],
      config_.joint_sign[i0 + 1] * snapshot.joint_velocity[i0 + 1]);

    VmcLegState vmc_state;
    if (!computeVmcLeg(
        config_.fivebar, config_.vmc, q, q_dot, foot_target, Eigen::Vector2d::Zero(),
        &vmc_state))
    {
      // 该腿不可装配 / 奇异：保持零力矩，另一条腿照常输出
      continue;
    }

    for (size_t k = 0; k < 2; ++k) {
      const size_t joint = i0 + k;
      // ∂α/∂q = joint_sign，所以 q 空间的力矩要乘同一个符号
      const double tau =
        config_.joint_sign[joint] * vmc_state.joint_torque(static_cast<Eigen::Index>(k));
      torque[joint] = std::clamp(
        tau * config_.joint_torque_sign[joint],
        -config_.joint_torque_max, config_.joint_torque_max);
    }
  }
  return torque;
}

std::array<double, 2> LegController::computeWheelSpeeds(
  const SafetyStatus & status, const StateSnapshot & snapshot, double u, double dt)
{
  std::array<double, 2> omega_ref{{0.0, 0.0}};

  const double balance_omega = std::clamp(
    u * config_.balance_torque_to_omega,
    -config_.max_balance_omega, config_.max_balance_omega);

  // 只有 RAW 且命令新鲜时才跟踪速度；降级状态速度目标归零
  const bool tracking = (status.state == ControlState::kRun) && status.command_fresh;
  const double v_des = tracking ? std::clamp(
    snapshot.command.linear_velocity,
    -config_.max_linear_velocity, config_.max_linear_velocity) : 0.0;
  const double w_des = tracking ? std::clamp(
    snapshot.command.yaw_rate, -config_.max_yaw_rate, config_.max_yaw_rate) : 0.0;

  const double drive[2] = {
    std::clamp(
      (v_des - 0.5 * config_.wheel_separation * w_des) / config_.wheel_radius,
      -config_.max_drive_omega, config_.max_drive_omega),
    std::clamp(
      (v_des + 0.5 * config_.wheel_separation * w_des) / config_.wheel_radius,
      -config_.max_drive_omega, config_.max_drive_omega)
  };

  // 一阶漏积分：平衡分量按轮速目标缓变，避免 LQR 输出突变直接打到速度环
  constexpr double kBalanceLeakPerS = 6.0;
  const double leak_rate = (config_.balance_leak_rate > 0.0) ? config_.balance_leak_rate : kBalanceLeakPerS;
  for (size_t i = 0; i < 2; ++i) {
    const double target = balance_omega + drive[i];
    const double alpha = std::clamp(leak_rate * std::max(0.0, dt), 0.0, 1.0);
    balance_omega_state_[i] += alpha * (target - balance_omega_state_[i]);
    omega_ref[i] = std::clamp(
      balance_omega_state_[i], -config_.max_wheel_omega, config_.max_wheel_omega);
  }
  return omega_ref;
}

}  // namespace leg_calc
