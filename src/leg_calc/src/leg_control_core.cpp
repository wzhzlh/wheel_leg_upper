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
  wheel_torque_state_ = {{0.0, 0.0}};
  com_height_cmd_ = com_height_target_;
  com_height_ = config_.model.com_length;
  yaw_rate_filt_ = 0.0;
  vel_i_ = 0.0;
  yaw_i_ = 0.0;
  x_ref_valid_ = false;
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
    updatePositionReference(snapshot, status, dt);
    out.u = computeBalanceTorque(snapshot, gain, status.command_fresh);
    out.joint_torque = computeJointTorques(snapshot);
    // 下位机运控模式要的“位置目标”：由足端目标 IK 得到，kp/kd 非 0 时才真正起作用
    out.joint_position_target = computeJointPositionTarget(snapshot);
    out.joint_target_valid = out.joint_position_target[0] != 0.0 ||
      out.joint_position_target[1] != 0.0 ||
      out.joint_position_target[2] != 0.0 ||
      out.joint_position_target[3] != 0.0;
    // 轮速参考只给仿真桥（sim_bridge 用它做速度环）与调试观测，实机不执行。
    // 必须排在力矩之前：它顺带更新 yaw_rate_filt_，computeWheelTorques 的偏航 P 要用。
    out.wheel_omega_ref = computeWheelSpeeds(status, snapshot, out.u, dt);
    // 主通道：轮子力矩（固件纯力矩直通，帧里 wheel[].torque → M2006 电流）
    out.wheel_torque_ref = config_.wheel_torque_enable ?
      computeWheelTorques(status, snapshot, out.u, dt) :
      std::array<double, 2>{{0.0, 0.0}};
  } else {
    // 未使能 / 故障：关节失力；固件没有位置环，轮子只能靠 −wheel_brake_damp·ω 阻尼
    // （阻尼项关掉时就是完全失力，坡上会溜车）
    balance_omega_state_ = {{0.0, 0.0}};
    vel_i_ = 0.0;
    yaw_i_ = 0.0;
    x_ref_valid_ = false;   // 退出平衡后重新锚定位置参考
    if (config_.wheel_torque_enable) {
      for (size_t i = 0; i < 2; ++i) {
        out.wheel_torque_ref[i] = std::clamp(
          -config_.wheel_brake_damp * snapshot.wheel_omega[i],
          -config_.wheel_torque_max, config_.wheel_torque_max);
      }
      wheel_torque_state_ = out.wheel_torque_ref;
    }
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
  const double prev_cmd = com_height_cmd_;
  com_height_cmd_ += std::clamp(com_height_target_ - com_height_cmd_, -step, step);
  // 腿长指令的实际变化率 [m/s]（正 = 腿在伸长）：作为 VMC 的足端速度前馈。
  // 指令已到位的稳态下它恒为 0，不会影响站立刚性。
  com_height_vel_ = (dt > 0.0) ? (com_height_cmd_ - prev_cmd) / dt : 0.0;
}

void LegController::updatePositionReference(
  const StateSnapshot & snapshot, const SafetyStatus & status, double dt)
{
  if (!x_ref_valid_) {
    x_ref_ = snapshot.x;   // 首次进入平衡：以当前轮端位置为基准
    x_ref_valid_ = true;
  }
  const bool tracking = (status.state == ControlState::kRun) && status.command_fresh;
  const double v_des = tracking ? std::clamp(
    snapshot.command.linear_velocity,
    -config_.max_linear_velocity, config_.max_linear_velocity) : 0.0;
  // 行驶时参考位置按指令速度积分（不是跟随实测）：位置误差 = 跟踪滞后，
  // 完整 LQR 的 x/ẋ 两项一起把速度收敛到 v_des。若直接跟随实测，位置项恒 0，
  // 只剩 k_x_dot 一项 —— 它和位置项配对时才稳定，单独工作会变成正反馈（实测 0.1 指令冲到 0.4 m/s）。
  // 停车后参考冻结，位置误差把残余溜车刹住（x 是绝对轮端转角，不能锚在 0）。
  if (std::abs(v_des) > 1e-3) {
    x_ref_ += v_des * std::max(0.0, dt);
  }
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
  const StateSnapshot & snapshot, const GainSchedule::Gain & gain, bool command_fresh) const
{
  WheelLegModel::State state;
  state << snapshot.x, snapshot.pitch, snapshot.x_dot, snapshot.pitch_rate;
  WheelLegModel::State ref = WheelLegModel::State::Zero();
  ref(kTheta) = snapshot.command.pitch_target;
  // 位置项：行驶时 x_ref 按指令速度积分；停车后冻结在最后一次参考位置，
  // 让 LQR 的 x 项把残余溜车刹住（Snapshot.x 是轮子多圈绝对转角×r，不能直接锚在 0）。
  ref(kX) = config_.wheel_torque_enable && x_ref_valid_ ? x_ref_ : snapshot.x;
  // 速度项必须保留：k_x_dot<0 时它和 θ/θ̇ 项一起把"车体速度"极点拉成稳定
  // （线性化 A−B·K 在去掉 x_dot 反馈后有两个零极点，任何扰动都会变成匀速漂移）。
  // 只有力矩通道（实机/仿真主通道）才由 LQR 做速度参考；速度通道交给 /wheel_target 差速。
  if (config_.wheel_torque_enable) {
    ref(kXdot) = command_fresh ? std::clamp(
      snapshot.command.linear_velocity,
      -config_.max_linear_velocity, config_.max_linear_velocity) : 0.0;
  } else {
    ref(kXdot) = snapshot.x_dot;
  }

  const double u = -(gain * (state - ref))(0, 0);
  return std::clamp(u, -config_.u_max, config_.u_max);
}

std::array<double, 4> LegController::computeJointPositionTarget(
  const StateSnapshot & snapshot) const
{
  std::array<double, 4> q_target{{0.0, 0.0, 0.0, 0.0}};
  const double foot_y_target = com_height_cmd_ - config_.leg_geometry.com_offset;
  const Eigen::Vector2d foot_target(config_.foot_target_x, foot_y_target);

  for (int leg = 0; leg < 2; ++leg) {
    const size_t i0 = static_cast<size_t>(2 * leg);
    // 用当前姿态当种子选装配分支，保证目标角与实测角连续（不会突然翻肘）
    Eigen::Vector2d q_seed = legJointAngles(snapshot, leg);
    Eigen::Vector2d alpha_beta;
    if (!config_.fivebar.inverseKinematics(foot_target, &alpha_beta, &q_seed)) {
      continue;   // 超出工作空间：保持 0，配合 kp=0 时无影响
    }
    // α/β = joint_sign·q + joint_offset  →  q = joint_sign·(α/β − joint_offset)
    q_target[i0] = config_.joint_sign[i0] * (alpha_beta(0) - config_.joint_offset[i0]);
    q_target[i0 + 1] =
      config_.joint_sign[i0 + 1] * (alpha_beta(1) - config_.joint_offset[i0 + 1]);
  }
  return q_target;
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

    // 足端目标速度：腿长指令在动的时候足端 y 必须同步移动。
    // 传 0 会让 kd_y 一直按 (0 − ẋ) 反向阻尼（0.30 m/s × kd_y=70 ≈ 21 N），
    // 把腿死死拖住 —— 改腿长又慢又抖，还向机身注入俯仰扰动。
    const Eigen::Vector2d foot_vel_target(0.0, com_height_vel_);

    VmcLegState vmc_state;
    if (!computeVmcLeg(
        config_.fivebar, config_.vmc, q, q_dot, foot_target, foot_vel_target, &vmc_state))
    {
      // 该腿不可装配 / 奇异：保持零力矩，另一条腿照常输出
      continue;
    }

    // 重力前馈：摆杆重量 m·g 由两条腿分担，每条 m·g/2 沿 +y（蹬地撑住机身）。
    // 抵消 VMC 的静态下蹲量 mg/(2·kp_y)（kp_y=2500 时约 3.7 mm）。
    Eigen::Vector2d foot_force = vmc_state.foot_force;
    if (config_.vmc_gravity_comp) {
      foot_force(1) += 0.5 * config_.model.pendulum_mass * config_.model.gravity;
    }
    const Eigen::Vector2d tau_vmc = vmc_state.jacobian.transpose() * foot_force;

    for (size_t k = 0; k < 2; ++k) {
      const size_t joint = i0 + k;
      // ∂α/∂q = joint_sign，所以 q 空间的力矩要乘同一个符号
      const double tau =
        config_.joint_sign[joint] * tau_vmc(static_cast<Eigen::Index>(k));
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
  // 只有仿真桥 sim_bridge 消费这条通道（它内部用 PI 力矩环跟踪 ω_ref）；
  // 实机固件是纯力矩直通，不读 wheel[].omega，实机执行见 computeWheelTorques。
  //   ω_ref = 平衡分量（LQR 力矩折算） + 差速行驶分量（v/r ± 0.5·L·ω/r）
  std::array<double, 2> omega_ref{{0.0, 0.0}};

  const double balance_omega = std::clamp(
    u * config_.balance_torque_to_omega,
    -config_.max_balance_omega, config_.max_balance_omega);

  // 只有 RUN 且命令新鲜时才跟踪速度；降级状态速度/偏航目标归零
  const bool tracking = (status.state == ControlState::kRun) && status.command_fresh;
  const double v_des = tracking ? std::clamp(
    snapshot.command.linear_velocity,
    -config_.max_linear_velocity, config_.max_linear_velocity) : 0.0;
  const double w_des = tracking ? std::clamp(
    snapshot.command.yaw_rate, -config_.max_yaw_rate, config_.max_yaw_rate) : 0.0;

  // 差速运动学：ω > 0 为左转 → 左轮慢、右轮快
  const double half_track = 0.5 * config_.wheel_separation;
  const double drive[2] = {
    std::clamp(
      (v_des - half_track * w_des) / config_.wheel_radius,
      -config_.max_drive_omega, config_.max_drive_omega),
    std::clamp(
      (v_des + half_track * w_des) / config_.wheel_radius,
      -config_.max_drive_omega, config_.max_drive_omega)
  };

  // 一阶漏积分：平衡分量缓变，避免 LQR 输出突变直接打到速度环
  constexpr double kBalanceLeakPerS = 6.0;
  const double leak_rate =
    (config_.balance_leak_rate > 0.0) ? config_.balance_leak_rate : kBalanceLeakPerS;
  const double alpha = std::clamp(leak_rate * std::max(0.0, dt), 0.0, 1.0);
  for (size_t i = 0; i < 2; ++i) {
    const double target = balance_omega + drive[i];
    balance_omega_state_[i] += alpha * (target - balance_omega_state_[i]);
    omega_ref[i] = std::clamp(
      balance_omega_state_[i], -config_.max_wheel_omega, config_.max_wheel_omega);
  }

  // 偏航角速度一阶低通（20 Hz）：力矩前馈通道的 k_yaw_p 要用，每周期更新一次。
  // IMU 200 Hz 更新，原始值直接进 P 会把高频噪声放大成俯仰扰动。
  const double yaw_alpha = std::clamp(dt / (dt + 0.008), 0.0, 1.0);
  yaw_rate_filt_ += yaw_alpha * (snapshot.yaw_rate - yaw_rate_filt_);
  return omega_ref;
}

std::array<double, 2> LegController::computeWheelTorques(
  const SafetyStatus & status, const StateSnapshot & snapshot, double u, double dt)
{
  // LQR 的 u 是**总轮轴力矩**（模型 inputMatrix 的 Bq = 1/r，即 u 产生的纵向广义力是 u/r），
  // 左右对称各分一半：τ_L0 = τ_R0 = u/2，两轮合力矩 = u，与模型一致。
  // u_max 因此也按“总力矩”理解（每轮实际 = u/2 ≤ u_max/2 ≤ wheel_torque_max）。
  // 下面叠加的速度项 / 偏航项按“每轮”定义（k_vel_p、k_yaw_p 是单轮系数），不再分半。
  std::array<double, 2> tau_ref{{0.5 * u, 0.5 * u}};

  // 只有 RUN 且命令新鲜时才跟踪速度/偏航；其它状态目标归零，P 项自然退化成阻尼
  const bool tracking = (status.state == ControlState::kRun) && status.command_fresh;
  const double v_des = tracking ? std::clamp(
    snapshot.command.linear_velocity,
    -config_.max_linear_velocity, config_.max_linear_velocity) : 0.0;
  const double w_des = tracking ? std::clamp(
    snapshot.command.yaw_rate, -config_.max_yaw_rate, config_.max_yaw_rate) : 0.0;

  // 线速度误差 P [N·m per m/s]，左右同相；积分项可选（补静摩擦死区）
  double tau_vel = config_.k_vel_p * (v_des - snapshot.x_dot);
  if (config_.k_vel_i > 0.0) {
    if (tracking) {
      vel_i_ = std::clamp(
        vel_i_ + config_.k_vel_i * (v_des - snapshot.x_dot) * std::max(0.0, dt),
        -0.5 * config_.wheel_torque_max, 0.5 * config_.wheel_torque_max);
    } else {
      vel_i_ = 0.0;
    }
    tau_vel += vel_i_;
  }

  // 偏航误差 P [N·m per rad/s]，左右反相；yaw_ff 是角速度前馈。
  // yaw_rate_filt_ 由 computeWheelSpeeds 在本周期已更新（IMU 200 Hz，原始值直接进 P 会抖）
  double tau_yaw = config_.yaw_ff * w_des + config_.k_yaw_p * (w_des - yaw_rate_filt_);
  if (config_.k_yaw_i > 0.0) {
    if (tracking) {
      // 偏航积分限幅用满 wheel_torque_max（而不是 0.5）：转向是**差模**，
      // 左右各 ±τ_yaw → 实际力矩差 2·τ_yaw，原地转向要突破 μ·N·L/2 ≈ 2.1 N·m，
      // 所以单侧积分必须能到 ~1 N·m，否则永远破不了静摩擦死区。
      yaw_i_ = std::clamp(
        yaw_i_ + config_.k_yaw_i * (w_des - yaw_rate_filt_) * std::max(0.0, dt),
        -config_.wheel_torque_max, config_.wheel_torque_max);
    } else {
      yaw_i_ = 0.0;
    }
    tau_yaw += yaw_i_;
  }

  // 正偏航 = 左转：右轮 +τ_yaw、左轮 −τ_yaw（两轮在世界系里是同一转向方向）
  tau_ref[0] = std::clamp(
    0.5 * u + tau_vel - tau_yaw, -config_.wheel_torque_max, config_.wheel_torque_max);
  tau_ref[1] = std::clamp(
    0.5 * u + tau_vel + tau_yaw, -config_.wheel_torque_max, config_.wheel_torque_max);

  // 一阶低通（沿用 balance_leak_rate）：500 Hz 力矩直通会把腿部/俯仰的 ~10 Hz 谐振激励起来，
  // 表现为 pitch_rate ±2 rad/s 的极限环。速度通道本来就被这条滤波平滑过，力矩通道也要一致。
  constexpr double kLeakDefault = 6.0;
  const double leak_rate =
    (config_.balance_leak_rate > 0.0) ? config_.balance_leak_rate : kLeakDefault;
  const double alpha = std::clamp(leak_rate * std::max(0.0, dt), 0.0, 1.0);
  for (size_t i = 0; i < 2; ++i) {
    wheel_torque_state_[i] += alpha * (tau_ref[i] - wheel_torque_state_[i]);
    tau_ref[i] = wheel_torque_state_[i];
  }
  return tau_ref;
}

}  // namespace leg_calc
