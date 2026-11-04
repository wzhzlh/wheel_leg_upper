// 控制核心（状态机门控 / VMC 力矩 / 轮子分配 / 超时）的单测
//
// 直接构造 StateSnapshot 驱动 LegController，不需要 ROS 运行环境。

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "leg_calc/leg_control_core.hpp"

namespace
{

constexpr double kHalfPi = 1.5707963267948966;

leg_calc::ControllerConfig makeConfig()
{
  leg_calc::ControllerConfig cfg;
  cfg.safety.startup_cycles = 3;   // 测试里 3 个周期即可进 RUN
  cfg.com_height_default = 0.40;   // = 0.30 足端 y + 0.10 com_offset
  cfg.foot_target_x = 0.05;
  return cfg;
}

leg_calc::StateSnapshot makeSnapshot(
  bool enable = true, double linear = 0.0, double yaw = 0.0)
{
  leg_calc::StateSnapshot s;
  s.imu_ok = true;
  s.imu_age = 0.005;
  s.joint_ok = true;
  s.joint_age = 0.005;
  s.wheel_ok = true;
  s.wheel_age = 0.005;
  s.command_seen = true;
  s.command_age = 0.010;

  s.pitch = 0.02;
  s.pitch_rate = 0.0;
  s.x = 0.0;
  s.x_dot = 0.0;

  s.joint_position = {{kHalfPi, kHalfPi, kHalfPi, kHalfPi}};
  s.joint_velocity = {{0.0, 0.0, 0.0, 0.0}};

  s.command.enable = enable;
  s.command.estop = false;
  s.command.linear_velocity = linear;
  s.command.yaw_rate = yaw;
  s.command.com_height = 0.40;
  return s;
}

/// 反复喂同一个快照直到进入期望状态（最多 50 个周期）
leg_calc::ControlOutput runUntil(
  leg_calc::LegController & controller, const leg_calc::StateSnapshot & s,
  leg_calc::ControlState expected)
{
  leg_calc::ControlOutput out;
  for (int i = 0; i < 50; ++i) {
    out = controller.step(s, 0.002);
    if (out.state == expected) {
      break;
    }
  }
  return out;
}

}  // namespace

TEST(LegControlCore, IdleOutputsNothing)
{
  leg_calc::LegController controller(makeConfig());
  const auto out = controller.step(makeSnapshot(false), 0.002);

  EXPECT_EQ(out.state, leg_calc::ControlState::kIdle);
  EXPECT_FALSE(out.balance_enabled);
  EXPECT_FALSE(out.joints_enabled);
  for (double tau : out.joint_torque) {
    EXPECT_DOUBLE_EQ(tau, 0.0);
  }
  for (double omega : out.wheel_omega_ref) {
    EXPECT_DOUBLE_EQ(omega, 0.0);
  }
}

TEST(LegControlCore, RunAfterStartupCycles)
{
  leg_calc::LegController controller(makeConfig());
  const auto out = runUntil(controller, makeSnapshot(true), leg_calc::ControlState::kRun);

  EXPECT_EQ(out.state, leg_calc::ControlState::kRun);
  EXPECT_TRUE(out.balance_enabled);
  EXPECT_TRUE(out.joints_enabled);

  // 足端目标 0.30 高于中位 0.2936，VMC 应有支撑力矩；且全部在限幅内
  bool any_torque = false;
  for (double tau : out.joint_torque) {
    EXPECT_LE(std::abs(tau), controller.config().joint_torque_max);
    any_torque = any_torque || std::abs(tau) > 1e-6;
  }
  EXPECT_TRUE(any_torque);
}

TEST(LegControlCore, JointTorqueIsClamped)
{
  auto cfg = makeConfig();
  cfg.joint_torque_max = 0.001;
  leg_calc::LegController controller(cfg);

  const auto out = runUntil(controller, makeSnapshot(true), leg_calc::ControlState::kRun);
  for (double tau : out.joint_torque) {
    EXPECT_LE(std::abs(tau), cfg.joint_torque_max + 1e-12);
  }
}

TEST(LegControlCore, WheelAllocationRespectsLimits)
{
  auto cfg = makeConfig();
  cfg.max_wheel_omega = 5.0;
  leg_calc::LegController controller(cfg);

  // 大速度 + 大转向：只允许在限幅内输出
  const auto out = runUntil(controller, makeSnapshot(true, 5.0, 5.0), leg_calc::ControlState::kRun);
  for (double omega : out.wheel_omega_ref) {
    EXPECT_LE(std::abs(omega), cfg.max_wheel_omega + 1e-9);
  }
}

TEST(LegControlCore, StraightDriveMakesBothWheelsSameDirection)
{
  leg_calc::LegController controller(makeConfig());
  const auto out = runUntil(controller, makeSnapshot(true, 0.3, 0.0), leg_calc::ControlState::kRun);

  // 速度通道只载"平衡 + 阻尼补偿"（直线），左右同值同向
  EXPECT_GT(out.wheel_omega_ref[0] * out.wheel_omega_ref[1], 0.0);
  EXPECT_DOUBLE_EQ(out.wheel_omega_ref[0], out.wheel_omega_ref[1]);
}

TEST(LegControlCore, WheelTorqueIsBalancePlusVelocityP)
{
  auto cfg = makeConfig();
  cfg.wheel_torque_enable = true;   // 实机主通道（固件纯力矩直通）
  cfg.k_vel_p = 1.0;
  leg_calc::LegController controller(cfg);
  const auto out = runUntil(controller, makeSnapshot(true, 0.3, 0.0), leg_calc::ControlState::kRun);

  // 挂空转（x_dot=0）给 0.3 m/s：τ = u + k_vel_p·v_des，左右同相、被 wheel_torque_max 限幅
  EXPECT_GT(out.wheel_torque_ref[0], 0.0);
  EXPECT_DOUBLE_EQ(out.wheel_torque_ref[0], out.wheel_torque_ref[1]);
  for (double tau : out.wheel_torque_ref) {
    EXPECT_LE(std::abs(tau), cfg.wheel_torque_max + 1e-12);
  }
}

TEST(LegControlCore, WheelTorqueDisabledWhenSwitchedOff)
{
  // 力矩通道默认开（固件纯力矩直通，实机执行就靠它）；显式关掉后必须恒为 0
  auto cfg = makeConfig();
  EXPECT_TRUE(cfg.wheel_torque_enable);
  cfg.wheel_torque_enable = false;
  leg_calc::LegController controller(cfg);
  const auto out = runUntil(controller, makeSnapshot(true, 0.3, 0.5), leg_calc::ControlState::kRun);

  for (double tau : out.wheel_torque_ref) {
    EXPECT_DOUBLE_EQ(tau, 0.0);
  }
}

TEST(LegControlCore, FarTravelDoesNotSaturateBalanceTorque)
{
  // 轮端 x 是轮子多圈绝对转角×r（无起点锚定）：跑出几十米后位置项不能把 u 顶到 u_max，
  // 否则姿态项被淹没 —— 现象就是“轮子在转，但摆动机身完全没有反应”。
  auto cfg = makeConfig();
  cfg.wheel_torque_enable = true;
  cfg.u_max = 1.0;
  cfg.k_coeffs << 0.0, 0.0, -1.18,
    0.0, 0.0, -4.52,
    0.0, 0.0, -2.00,
    0.0, 0.0, -0.28;
  leg_calc::LegController controller(cfg);

  auto far = makeSnapshot(true, 0.0, 0.0);
  far.x = -27.0;          // 已经滚了 27 m
  far.x_dot = 0.0;
  far.pitch = 0.0;
  far.pitch_rate = 0.0;
  const auto level = runUntil(controller, far, leg_calc::ControlState::kRun);
  EXPECT_LT(std::abs(level.u), 0.05);      // 平放：位置项零误差，不该出力

  auto tilted = far;
  tilted.pitch = 0.15;
  const auto out = controller.step(tilted, 0.002);
  EXPECT_GT(std::abs(out.u), 0.5);         // 姿态变化必须真的改变 u（0.15×4.52≈0.68）

  // 速度项：力矩通道下 LQR 的 x_dot 反馈是完整状态环的一部分，必须和 x_ref 位置项
  // 配对使用（单独把 x/ẋ 误差清零会剩两个零极点，任何扰动都变成匀速漂移）。
  // 单步看 ẋ>0 会给出正力矩，但这里只要求“不能因为 0.4 m/s 就饱和”。
  auto moving = far;
  moving.x_dot = 0.4;
  const auto roll = controller.step(moving, 0.002);
  EXPECT_LT(std::abs(roll.u), cfg.u_max);   // 不应该仅因速度就顶到限幅
}

TEST(LegControlCore, LeftTurnMakesRightWheelMoreTorque)
{
  auto cfg = makeConfig();
  cfg.wheel_torque_enable = true;
  cfg.yaw_ff = 0.12;
  cfg.k_yaw_p = 0.15;
  leg_calc::LegController controller(cfg);
  const auto out = runUntil(controller, makeSnapshot(true, 0.0, 1.0), leg_calc::ControlState::kRun);

  // ω>0 为左转：右轮力矩 > 左轮（力矩差速）
  EXPECT_LT(out.wheel_torque_ref[0], out.wheel_torque_ref[1]);
}

TEST(LegControlCore, WheelTorqueRespectsLimit)
{
  auto cfg = makeConfig();
  cfg.wheel_torque_enable = true;
  cfg.wheel_torque_max = 0.2;
  leg_calc::LegController controller(cfg);
  const auto out =
    runUntil(controller, makeSnapshot(true, 20.0, 20.0), leg_calc::ControlState::kRun);

  for (double tau : out.wheel_torque_ref) {
    EXPECT_LE(std::abs(tau), cfg.wheel_torque_max + 1e-12);
  }
}

TEST(LegControlCore, LeftTurnSlowsLeftWheel)
{
  leg_calc::LegController controller(makeConfig());
  const auto out = runUntil(controller, makeSnapshot(true, 0.3, 1.0), leg_calc::ControlState::kRun);

  // ω>0 为左转：左轮 = v/r − W/2·ω/r，应小于右轮
  EXPECT_LT(out.wheel_omega_ref[0], out.wheel_omega_ref[1]);
}

TEST(LegControlCore, CommandTimeoutDegradesButKeepsTorque)
{
  leg_calc::LegController controller(makeConfig());
  (void)runUntil(controller, makeSnapshot(true, 0.3, 0.0), leg_calc::ControlState::kRun);

  auto stale = makeSnapshot(true, 0.3, 0.0);
  stale.command_age = controller.config().safety.command_timeout + 0.05;
  const auto out = controller.step(stale, 0.002);

  EXPECT_EQ(out.state, leg_calc::ControlState::kDegraded);
  EXPECT_FALSE(out.command_fresh);
  EXPECT_TRUE(out.balance_enabled);

  // 降级时速度目标归零：只剩平衡分量，且不会反向
  for (double omega : out.wheel_omega_ref) {
    EXPECT_LT(std::abs(omega), 0.3 / controller.config().wheel_radius);
  }
}

TEST(LegControlCore, LongCommandLossFaultsAndDropsTorque)
{
  leg_calc::LegController controller(makeConfig());
  (void)runUntil(controller, makeSnapshot(true), leg_calc::ControlState::kRun);

  auto stale = makeSnapshot(true);
  stale.command_age = controller.config().safety.command_fault_timeout + 0.1;
  const auto out = controller.step(stale, 0.002);

  EXPECT_EQ(out.state, leg_calc::ControlState::kFault);
  EXPECT_NE(out.faults & leg_calc::kFaultCommand, 0u);
  EXPECT_FALSE(out.joints_enabled);
  for (double tau : out.joint_torque) {
    EXPECT_DOUBLE_EQ(tau, 0.0);
  }
}

TEST(LegControlCore, SensorLossFaults)
{
  leg_calc::LegController controller(makeConfig());
  (void)runUntil(controller, makeSnapshot(true), leg_calc::ControlState::kRun);

  auto bad = makeSnapshot(true);
  bad.joint_age = 0.5;
  const auto out = controller.step(bad, 0.002);

  EXPECT_EQ(out.state, leg_calc::ControlState::kFault);
  EXPECT_NE(out.faults & leg_calc::kFaultJointState, 0u);
  EXPECT_FALSE(out.balance_enabled);
}

TEST(LegControlCore, WheelFeedbackLossFaults)
{
  leg_calc::LegController controller(makeConfig());
  (void)runUntil(controller, makeSnapshot(true), leg_calc::ControlState::kRun);

  auto bad = makeSnapshot(true);
  bad.wheel_age = 0.5;  // 轮反馈超时：x / x_dot 不可信
  const auto out = controller.step(bad, 0.002);

  EXPECT_EQ(out.state, leg_calc::ControlState::kFault);
  EXPECT_NE(out.faults & leg_calc::kFaultWheelState, 0u);
  EXPECT_FALSE(out.balance_enabled);
}

TEST(LegControlCore, FailedGainSolveStopsOutput)
{
  leg_calc::LegController controller(makeConfig());
  ASSERT_EQ(
    runUntil(controller, makeSnapshot(true), leg_calc::ControlState::kRun).state,
    leg_calc::ControlState::kRun);

  auto cfg = makeConfig();
  cfg.gain_mode = leg_calc::GainMode::kOnlineLqr;
  cfg.Q.setZero();
  controller.setConfig(cfg);

  const auto out = controller.step(makeSnapshot(true), 0.002);
  EXPECT_EQ(out.state, leg_calc::ControlState::kFault);
  EXPECT_NE(out.faults & leg_calc::kFaultLqr, 0u);
  EXPECT_FALSE(out.balance_enabled);
  EXPECT_DOUBLE_EQ(out.u, 0.0);
}

TEST(LegControlCore, EstopIsHighestPriority)
{
  leg_calc::LegController controller(makeConfig());
  (void)runUntil(controller, makeSnapshot(true), leg_calc::ControlState::kRun);

  auto estop = makeSnapshot(true);
  estop.command.estop = true;
  const auto out = controller.step(estop, 0.002);

  EXPECT_EQ(out.state, leg_calc::ControlState::kFault);
  EXPECT_NE(out.faults & leg_calc::kFaultEstop, 0u);
}

TEST(LegControlCore, ComHeightRampsTowardTarget)
{
  leg_calc::LegController controller(makeConfig());
  (void)runUntil(controller, makeSnapshot(true), leg_calc::ControlState::kRun);

  auto cmd = makeSnapshot(true);
  cmd.command.com_height = 0.45;
  const double before = controller.comHeightCommand();
  (void)controller.step(cmd, 0.002);

  const double after = controller.comHeightCommand();
  EXPECT_GT(after, before);
  EXPECT_LE(after, before + controller.config().com_height_rate * 0.002 + 1e-9);
  EXPECT_LE(after, controller.config().com_height_max);
}

// 关节角标定：α/β = joint_sign·q + joint_offset。标定前后若映射到同一个五杆角，
// 腿长估计与关节力矩必须完全一致（右腿靠 joint_sign=-1 抵消轴反向）。
TEST(LegControlCore, JointCalibrationMapsToSameFiveBarAngles)
{
  leg_calc::ControllerConfig plain = makeConfig();
  leg_calc::ControllerConfig shifted = makeConfig();
  shifted.joint_sign = {{-1.0, -1.0, -1.0, -1.0}};
  shifted.joint_offset = {{kHalfPi, kHalfPi, kHalfPi, kHalfPi}};

  auto s_plain = makeSnapshot(true);
  auto s_shifted = makeSnapshot(true);
  for (int i = 0; i < 4; ++i) {
    // -(-q) + 90° = q + 90°
    s_shifted.joint_position[static_cast<size_t>(i)] =
      -s_plain.joint_position[static_cast<size_t>(i)];
  }
  // 让两边映射出的五杆角相同：plain 用 q+90°，shifted 用 -q 再 +90°
  for (int i = 0; i < 4; ++i) {
    s_plain.joint_position[static_cast<size_t>(i)] += kHalfPi;
  }

  // 该用例验证纯 VMC 的反对称性；重力前馈由 VmcGravityFeedforward 单独覆盖
  plain.vmc_gravity_comp = false;
  shifted.vmc_gravity_comp = false;
  leg_calc::LegController a(plain);
  leg_calc::LegController b(shifted);
  const auto out_a = runUntil(a, s_plain, leg_calc::ControlState::kRun);
  const auto out_b = runUntil(b, s_shifted, leg_calc::ControlState::kRun);

  EXPECT_NEAR(out_b.com_height, out_a.com_height, 1e-12);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(
      out_b.joint_torque[static_cast<size_t>(i)],
      -out_a.joint_torque[static_cast<size_t>(i)], 1e-9);
  }
}

TEST(LegControlCore, VmcGravityFeedforwardAddsSupportTorque)
{
  // 足端 m·g/2 前馈经 Jᵀ 映射后应产生明显的支撑力矩（消除静态下蹲）
  auto on = makeConfig();
  on.vmc_gravity_comp = true;
  auto off = makeConfig();
  off.vmc_gravity_comp = false;

  leg_calc::LegController c_on(on);
  leg_calc::LegController c_off(off);
  const auto s = makeSnapshot(true);
  const auto r_on = runUntil(c_on, s, leg_calc::ControlState::kRun);
  const auto r_off = runUntil(c_off, s, leg_calc::ControlState::kRun);

  double diff = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    diff += std::abs(r_on.joint_torque[i] - r_off.joint_torque[i]);
  }
  // 单腿 m·g/2 = 1.904×9.81/2 ≈ 9.34 N，乘雅可比后四关节合计应有 1 N·m 量级
  EXPECT_GT(diff, 0.5);
}

TEST(LegControlCore, HeightRampStaysWithinTorqueLimits)
{
  // 腿长斜坡期间：VMC 拿到足端速度前馈，力矩应保持有限、不超限、不来回翻转
  auto cfg = makeConfig();
  cfg.vmc.kp = Eigen::Vector2d(300.0, 2500.0);
  cfg.vmc.kd = Eigen::Vector2d(22.0, 70.0);
  cfg.com_height_rate = 0.50;
  leg_calc::LegController controller(cfg);

  auto s = makeSnapshot(true);
  s.command.com_height = 0.30;   // 低于默认 0.40，腿要收缩
  (void)runUntil(controller, s, leg_calc::ControlState::kRun);

  double max_abs = 0.0;
  for (int i = 0; i < 100; ++i) {   // 0.2 s
    const auto out = controller.step(s, 0.002);
    for (double tau : out.joint_torque) {
      EXPECT_TRUE(std::isfinite(tau));
      max_abs = std::max(max_abs, std::abs(tau));
    }
  }
  EXPECT_LE(max_abs, cfg.joint_torque_max);
  // 0.5 m/s × 0.2 s = 0.1 m 的斜坡上限，指令应从 0.40 明显下降
  EXPECT_LT(controller.comHeightCommand(), 0.40);
}

TEST(LegControlCore, YawIntegralBuildsDifferentialTorque)
{
  // 原地转向：w_des=1、陀螺反馈 0 时，k_yaw_p + k_yaw_i 必须把差模推过摩擦阈值（≈2.1 N·m）
  auto cfg = makeConfig();
  cfg.wheel_torque_enable = true;
  cfg.k_yaw_p = 1.2;
  cfg.k_yaw_i = 0.6;
  cfg.wheel_torque_max = 1.0;
  cfg.max_yaw_rate = 1.0;
  leg_calc::LegController controller(cfg);

  auto s = makeSnapshot(true, 0.0, 1.0);
  s.yaw_rate = 0.0;
  leg_calc::ControlOutput out;
  for (int i = 0; i < 200; ++i) {   // 0.4 s
    out = controller.step(s, 0.002);
  }
  ASSERT_EQ(out.state, leg_calc::ControlState::kRun);
  const double diff = out.wheel_torque_ref[1] - out.wheel_torque_ref[0];
  EXPECT_GT(diff, 1.5);   // 差模应朝 2.0 N·m（±wheel_torque_max）增长
}

TEST(LegControlCore, OnlineLqrSolvesAcrossHeightRange)
{
  // 部署用的是 online_lqr + q=[100,1050,10,6] + R=1.25（u 为总力矩语义，左右各 u/2）；
  // 节点启动时会对 h∈[h_min,h_max] 扫 11 个点做同样的稳定性校验，这里提前钉住，
  // 避免改 Q/R 后实机起不来。
  auto cfg = makeConfig();
  cfg.gain_mode = leg_calc::GainMode::kOnlineLqr;
  cfg.Q = Eigen::Vector4d(100.0, 1050.0, 10.0, 6.0).asDiagonal();
  cfg.R = Eigen::MatrixXd::Constant(1, 1, 1.25);
  cfg.h_min = 0.06;
  cfg.h_max = 0.12;

  leg_calc::GainSchedule gains(cfg.model, cfg.gain_mode);
  gains.setWeights(cfg.Q, cfg.R);
  gains.setHeightLimits(cfg.h_min, cfg.h_max);

  for (int i = 0; i <= 10; ++i) {
    const double h = cfg.h_min + (cfg.h_max - cfg.h_min) * static_cast<double>(i) / 10.0;
    const auto g = gains.gainAt(h);
    EXPECT_TRUE(gains.lastSolveOk()) << "h=" << h;
    EXPECT_TRUE(g.allFinite()) << "h=" << h;
    // k_theta 必须为负（前倾 → 正力矩）且幅值足够撑住倒立摆；
    // 上限按“总力矩”语义放宽（K 是每轮的两倍，u_max=0.80 时饱和角 ≈1.2°）
    EXPECT_LT(g(0, 1), -1.0) << "h=" << h;
    EXPECT_GT(g(0, 1), -45.0) << "h=" << h;
  }
}
