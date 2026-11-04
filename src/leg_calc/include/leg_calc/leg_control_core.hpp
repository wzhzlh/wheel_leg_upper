#ifndef LEG_CALC__LEG_CONTROL_CORE_HPP_
#define LEG_CALC__LEG_CONTROL_CORE_HPP_

// 控制计算核心（纯算法，不依赖 ROS）
//
// 一个周期内顺序完成：
//   ① 状态机（SafetySupervisor）判定本周期能做什么
//   ② Pitch LQR：u = −K(h)·(x − x_ref)，限幅 u_max
//   ③ 左右腿 VMC：足端弹簧阻尼 → τ = JᵀF，逐关节限幅
//   ④ 轮子分配：
//        力矩通道 wheel_torque_ref = 平衡 u + 速度误差 P ± 偏航误差 P（实机主通道，
//        下位机不做闭环，直接把 N·m 换成电流）；
//        速度通道 wheel_omega_ref = 平衡/行驶映射成轮速（保留给有速度环的固件与仿真）
//
// 输入 StateSnapshot（带各通道年龄），输出 ControlOutput；
// 所有“什么时候能做”的判断都在这里，ROS 侧只负责搬数据。

#include "leg_calc/control_state.hpp"
#include "leg_calc/leg_control_types.hpp"

namespace leg_calc
{

class LegController
{
public:
  LegController() = default;
  explicit LegController(const ControllerConfig & config);

  /// 周期边界切换配置（会重建 K(h) 调度器并限幅当前腿长目标）
  void setConfig(const ControllerConfig & config);
  const ControllerConfig & config() const {return config_;}

  /// 回到 INIT（例如重新使能前的复位）
  void reset();

  /// 推进一个控制周期
  ControlOutput step(const StateSnapshot & snapshot, double dt);

  /// 当前目标质心高度指令（斜坡后）[m]，调试/测试用
  double comHeightCommand() const {return com_height_cmd_;}

private:
  void updateComHeightTarget(const RobotCommand & command, double dt);
  /// 速度/位置参考：首次使能锚定，行驶时按指令速度积分，停车后冻结
  void updatePositionReference(
    const StateSnapshot & snapshot, const SafetyStatus & status, double dt);
  double computeComHeight(const StateSnapshot & snapshot) const;
  /// 原始关节角 → 五杆角 q = [α, β]（按 joint_sign / joint_offset 标定）
  Eigen::Vector2d legJointAngles(const StateSnapshot & snapshot, int leg) const;
  double computeBalanceTorque(
    const StateSnapshot & snapshot, const GainSchedule::Gain & gain,
    bool command_fresh) const;
  std::array<double, 4> computeJointTorques(const StateSnapshot & snapshot) const;
  /// 足端目标 → 五杆 IK → 关节坐标系的目标角（给下位机运控模式做位置目标用）
  std::array<double, 4> computeJointPositionTarget(const StateSnapshot & snapshot) const;
  std::array<double, 2> computeWheelSpeeds(
    const SafetyStatus & status, const StateSnapshot & snapshot, double u, double dt);
  /// 轮轴力矩指令（左, 右）= 平衡 u + 速度误差 P ± 偏航误差 P，限幅 wheel_torque_max
  std::array<double, 2> computeWheelTorques(
    const SafetyStatus & status, const StateSnapshot & snapshot, double u, double dt);

  ControllerConfig config_{};
  GainSchedule gains_{};
  SafetySupervisor safety_{};
  bool initialized_{false};

  double com_height_cmd_{0.40};
  double com_height_target_{0.40};
  std::array<double, 2> balance_omega_state_{{0.0, 0.0}};
  /// 力矩前馈通道的一阶低通状态 [N·m]：500 Hz 直通会激励腿部/俯仰谐振
  std::array<double, 2> wheel_torque_state_{{0.0, 0.0}};
  double com_height_{0.30};
  /// 腿长指令的实际变化率 [m/s]（正 = 伸长）：VMC 足端速度前馈用
  double com_height_vel_{0.0};
  /// 轮端位置参考 [m]（只在力矩通道用）：首次使能时锚定，行驶时按指令速度积分，停车后冻结
  double x_ref_{0.0};
  bool x_ref_valid_{false};
  /// 偏航角速度反馈的低通值 [rad/s]（IMU 角速度 200 Hz 更新，直接进 P 会抖）
  double yaw_rate_filt_{0.0};
  /// 力矩前馈模式（wheel_torque_enable=true）的速度/偏航积分微调量（N·m）
  double vel_i_{0.0};
  double yaw_i_{0.0};
};

}  // namespace leg_calc

#endif  // LEG_CALC__LEG_CONTROL_CORE_HPP_
