// 控制状态机 / 安全监督的单测：用假快照驱动，不需要 ROS 运行环境

#include <gtest/gtest.h>

#include "leg_calc/control_state.hpp"

namespace
{

using leg_calc::ControlState;
using leg_calc::SafetyInputs;
using leg_calc::SafetyLimits;
using leg_calc::SafetyStatus;
using leg_calc::SafetySupervisor;

/// 健康快照：IMU / 关节都新鲜，命令 10 ms 前刚收到，姿态接近直立
SafetyInputs healthyInputs()
{
  SafetyInputs in;
  in.imu_ok = true;
  in.imu_age = 0.005;
  in.joint_ok = true;
  in.joint_age = 0.005;
  in.wheel_ok = true;
  in.wheel_age = 0.005;
  in.command_seen = true;
  in.command_age = 0.010;
  in.pitch = 0.02;
  in.pitch_rate = 0.1;
  in.estop = false;
  in.enable_request = false;
  return in;
}

/// 连续喂 count 个周期
SafetyStatus run(SafetySupervisor & sup, const SafetyInputs & in, int count)
{
  SafetyStatus status;
  for (int i = 0; i < count; ++i) {
    status = sup.update(in);
  }
  return status;
}

}  // namespace

TEST(ControlState, InitGoesToIdleWithoutEnable)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  const SafetyStatus status = sup.update(in);

  EXPECT_EQ(status.state, ControlState::kIdle);
  EXPECT_FALSE(status.balance_enabled);
  EXPECT_FALSE(status.joints_enabled);
}

TEST(ControlState, RunRequiresSustainedHealth)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.enable_request = true;

  SafetyStatus status = run(sup, in, sup.limits().startup_cycles - 1);

  EXPECT_EQ(status.state, ControlState::kInit);
  EXPECT_FALSE(status.balance_enabled);

  status = sup.update(in);
  EXPECT_EQ(status.state, ControlState::kRun);
  EXPECT_TRUE(status.balance_enabled);
  EXPECT_TRUE(status.joints_enabled);
}

TEST(ControlState, HealthInterruptionResetsStartupCounter)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.enable_request = true;

  (void)run(sup, in, sup.limits().startup_cycles / 2);

  SafetyInputs bad = in;
  bad.joint_age = 0.5;  // 关节反馈超时
  (void)sup.update(bad);

  const SafetyStatus status = run(sup, in, sup.limits().startup_cycles - 1);
  EXPECT_EQ(status.state, ControlState::kInit);
}

TEST(ControlState, CommandTimeoutDegradesButKeepsBalance)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.enable_request = true;
  (void)run(sup, in, sup.limits().startup_cycles);

  SafetyInputs stale = in;
  stale.command_age = sup.limits().command_timeout + 0.05;
  const SafetyStatus degraded = sup.update(stale);

  EXPECT_EQ(degraded.state, ControlState::kDegraded);
  EXPECT_TRUE(degraded.balance_enabled);   // 降级仍保留平衡
  EXPECT_FALSE(degraded.command_fresh);    // 但速度目标必须归零

  const SafetyStatus recovered = sup.update(in);
  EXPECT_EQ(recovered.state, ControlState::kRun);
}

TEST(ControlState, LongCommandLossFaults)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.enable_request = true;
  (void)run(sup, in, sup.limits().startup_cycles);

  SafetyInputs stale = in;
  stale.command_age = sup.limits().command_fault_timeout + 0.1;
  const SafetyStatus status = sup.update(stale);

  EXPECT_EQ(status.state, ControlState::kFault);
  EXPECT_NE(status.faults & leg_calc::kFaultCommand, 0u);
  EXPECT_FALSE(status.balance_enabled);
}

TEST(ControlState, AttitudeFaultAutoRecoversWhenUpright)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.enable_request = true;
  (void)run(sup, in, sup.limits().startup_cycles);

  SafetyInputs tilted = in;
  tilted.pitch = sup.limits().max_pitch + 0.1;
  const SafetyStatus fault = sup.update(tilted);

  EXPECT_EQ(fault.state, ControlState::kFault);
  EXPECT_NE(fault.faults & leg_calc::kFaultAttitude, 0u);
  // 触发那一周期仍然失力（该周期按 RUN 判定，硬故障成立）
  EXPECT_FALSE(fault.balance_enabled);

  // 之后的停机周期仍保持平衡输出（否则关节一失力就会真的倒下去，没有恢复窗口）
  const SafetyStatus still_fault = sup.update(tilted);
  EXPECT_EQ(still_fault.state, ControlState::kFault);
  EXPECT_TRUE(still_fault.balance_enabled);

  // 倒伏（超过 2.5 倍门限）后不再保平衡，直接失力
  SafetyInputs fallen = tilted;
  fallen.pitch = 2.6 * sup.limits().max_pitch;
  const SafetyStatus fallen_status = sup.update(fallen);
  EXPECT_EQ(fallen_status.state, ControlState::kFault);
  EXPECT_FALSE(fallen_status.balance_enabled);

  // 姿态回正、角速度收敛、上层仍使能 → 自动回 RUN（不需要手动复位）
  SafetyInputs upright = in;
  upright.enable_request = true;
  const SafetyStatus recovered = run(sup, upright, 10);
  EXPECT_EQ(recovered.state, ControlState::kRun);
  EXPECT_TRUE(recovered.balance_enabled);
}

TEST(ControlState, AttitudeFaultStillClearedByReleasingEnable)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.enable_request = true;
  (void)run(sup, in, sup.limits().startup_cycles);

  SafetyInputs tilted = in;
  tilted.pitch = sup.limits().max_pitch + 0.1;
  EXPECT_EQ(sup.update(tilted).state, ControlState::kFault);

  // 撤掉使能后回到待命（手动复位路径保留）
  SafetyInputs released = in;
  released.enable_request = false;
  EXPECT_EQ(sup.update(released).state, ControlState::kIdle);
}

TEST(ControlState, AttitudeOutOfRangeWhileDisabledIsNotAFault)
{
  // 失能时机器人是被动的：翻倒在所难免，不该刷 FAULT
  // （否则仿真一上电就是 Idle ↔ Fault 反复跳动）
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.enable_request = false;
  (void)sup.update(in);  // Init → Idle

  SafetyInputs fallen = in;
  fallen.pitch = 1.2;
  fallen.pitch_rate = -12.0;
  const SafetyStatus status = sup.update(fallen);

  EXPECT_EQ(status.state, ControlState::kIdle);
  EXPECT_EQ(status.faults, leg_calc::kFaultNone);
  EXPECT_FALSE(status.balance_enabled);
}

TEST(ControlState, TiltedRobotRefusesToStandUp)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.enable_request = false;
  (void)sup.update(in);  // Init → Idle

  SafetyInputs fallen = in;   // 使能请求还在，但已经躺倒
  fallen.enable_request = true;
  fallen.pitch = sup.limits().max_pitch + 0.2;

  const SafetyStatus blocked = run(sup, fallen, sup.limits().startup_cycles * 2);

  EXPECT_EQ(blocked.state, ControlState::kIdle);   // 不站起，也不报故障
  EXPECT_EQ(blocked.faults, leg_calc::kFaultNone);
  EXPECT_FALSE(blocked.joints_enabled);

  // 姿态扶正后才允许进入 RUN
  SafetyInputs upright = in;
  upright.enable_request = true;
  const SafetyStatus recovered = run(sup, upright, sup.limits().startup_cycles);
  EXPECT_EQ(recovered.state, ControlState::kRun);
}

TEST(ControlState, EstopDominates)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.enable_request = true;
  (void)run(sup, in, sup.limits().startup_cycles);

  SafetyInputs estop = in;
  estop.estop = true;
  estop.enable_request = false;
  const SafetyStatus status = sup.update(estop);

  EXPECT_EQ(status.state, ControlState::kFault);
  EXPECT_NE(status.faults & leg_calc::kFaultEstop, 0u);
  EXPECT_FALSE(status.joints_enabled);
}

TEST(ControlState, SensorLossBeforeStartupStaysInInit)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.imu_ok = false;
  in.imu_age = 1.0;

  const SafetyStatus status = sup.update(in);

  EXPECT_EQ(status.state, ControlState::kInit);
  EXPECT_NE(status.faults & leg_calc::kFaultImu, 0u);
  EXPECT_FALSE(status.balance_enabled);
}

TEST(ControlState, LqrFailureLatchesFault)
{
  SafetySupervisor sup;
  SafetyInputs in = healthyInputs();
  in.enable_request = true;
  (void)run(sup, in, sup.limits().startup_cycles);

  in.lqr_ok = false;
  const SafetyStatus fault = sup.update(in);
  EXPECT_EQ(fault.state, ControlState::kFault);
  EXPECT_NE(fault.faults & leg_calc::kFaultLqr, 0u);
  EXPECT_FALSE(fault.balance_enabled);
}
