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
  cfg.max_drive_omega = 4.0;
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

  EXPECT_GT(out.wheel_omega_ref[0] * out.wheel_omega_ref[1], 0.0);
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
