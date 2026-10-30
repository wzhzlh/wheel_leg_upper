#ifndef LEG_CALC__LEG_CONTROL_CORE_HPP_
#define LEG_CALC__LEG_CONTROL_CORE_HPP_

// 控制计算核心（纯算法，不依赖 ROS）
//
// 一个周期内顺序完成：
//   ① 状态机（SafetySupervisor）判定本周期能做什么
//   ② Pitch LQR：u = −K(h)·(x − x_ref)，限幅 u_max
//   ③ 左右腿 VMC：足端弹簧阻尼 → τ = JᵀF，逐关节限幅
//   ④ 轮子分配：平衡分量 + 行驶/转向分量，统一按优先级限幅
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
  double computeComHeight(const StateSnapshot & snapshot) const;
  /// 原始关节角 → 五杆角 q = [α, β]（按 joint_sign / joint_offset 标定）
  Eigen::Vector2d legJointAngles(const StateSnapshot & snapshot, int leg) const;
  double computeBalanceTorque(
    const StateSnapshot & snapshot, const GainSchedule::Gain & gain) const;
  std::array<double, 4> computeJointTorques(const StateSnapshot & snapshot) const;
  std::array<double, 2> computeWheelSpeeds(
    const SafetyStatus & status, const StateSnapshot & snapshot, double u, double dt);

  ControllerConfig config_{};
  GainSchedule gains_{};
  SafetySupervisor safety_{};
  bool initialized_{false};

  double com_height_cmd_{0.40};
  double com_height_target_{0.40};
  std::array<double, 2> balance_omega_state_{{0.0, 0.0}};
  double com_height_{0.30};
};

}  // namespace leg_calc

#endif  // LEG_CALC__LEG_CONTROL_CORE_HPP_
