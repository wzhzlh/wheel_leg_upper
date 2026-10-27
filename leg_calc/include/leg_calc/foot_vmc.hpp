#ifndef LEG_CALC__FOOT_VMC_HPP_
#define LEG_CALC__FOOT_VMC_HPP_

// 足端虚拟模型控制（VMC）：五杆闭链腿的笛卡尔空间弹簧-阻尼 + 雅可比力矩映射
//
// 坐标系（与实机机械图一致，单位 m / rad）：
//
//            y=0  O --------- D   ← 机身上两个髋关节，相距 l5
//                 |  α      β |
//                 A           C  ← 前/后肘关节（被动）
//                  \         /
//                   \       /
//                       B        ← 足端（轮心），|AB|=l2，|CB|=l3
//                       ↓ y 向下
//
//   O = (0, 0)，D = (l5, 0)
//   A = O + l1 * (cos α, sin α)      l1 = O→A
//   C = D + l4 * (cos β, sin β)      l4 = D→C
//   B = 圆(A, l2) 与 圆(C, l3) 的交点（由 foot_branch 选定装配分支）
//
//   α、β 均以 +x 轴为 0，转向 +y（屏幕下方）为正；x 向前，y 向下。
//   关节向量 q = [α, β]，两腿共用同一套几何，左右腿由固件 joint_calib 处理方向。
//
// 控制律（逐轴 PD，等价于在足端挂两根虚拟弹簧 + 两个阻尼器）：
//   F      = Kp·(x_d − x) + Kd·(ẋ_d − ẋ)        足端期望力 [N]
//   x      = T(q)                                 正运动学
//   ẋ      = J(q)·q̇                               正雅可比
//   q̇      = J(q)⁻¹·ẋ                             逆雅可比（每腿 2 DOF，J 为 2×2）
//   τ      = J(q)ᵀ·F                              等效力矩（虚功原理，VMC 核心）
//
// 说明：本腿是 2 DOF 闭链，J 是方阵，τ = JᵀF 已给出全部关节力矩，无需
//       (JᵀJ)⁻¹ 这类伪逆；只有在「足端力 → 关节加速度」时才需要 J⁻¹（配合 J̇q̇）。

#include <Eigen/Dense>

namespace leg_calc
{

/// 五杆腿几何参数（见文件头的坐标定义）
struct FiveBarGeometry
{
  double l1{0.10};  ///< O→A 前摆臂
  double l2{0.20};  ///< A→B 前连杆
  double l3{0.20};  ///< C→B 后连杆
  double l4{0.10};  ///< D→C 后摆臂
  double l5{0.10};  ///< O→D 两髋间距

  /// 正运动学装配分支：+1 足端在 A→C 连线下方（图中装配），-1 取上方
  int foot_branch{1};
  /// 逆运动学前肘分支：+1 前肘 A 在 O→B 连线左侧（图中装配），-1 取右侧
  int front_branch{1};
  /// 逆运动学后肘分支：+1 后肘 C 在 D→B 连线右侧（图中装配），-1 取左侧
  int rear_branch{1};

  /// 长度是否合法（正数 + 五杆能闭合）
  bool isValid() const;

  Eigen::Vector2d hipNear() const {return Eigen::Vector2d(0.0, 0.0);}
  Eigen::Vector2d hipFar() const {return Eigen::Vector2d(l5, 0.0);}

  /// 前肘 A(α)、后肘 C(β)
  Eigen::Vector2d frontElbow(double alpha) const;
  Eigen::Vector2d rearElbow(double beta) const;

  /// 正运动学：q = [α, β] → 足端位置；不可装配时返回 false
  bool footPosition(const Eigen::Vector2d & q, Eigen::Vector2d * foot) const;

  /// 正雅可比 J = ∂x/∂q（2×2）；不可装配或奇异时返回 false
  bool jacobian(const Eigen::Vector2d & q, Eigen::Matrix2d * jac) const;

  /**
   * @brief 逆运动学：足端位置 → q = [α, β]；超出工作空间时返回 false
   *
   * 每个肘关节各有两个解（前/后 × 左/右），共 4 种装配；选哪一组由
   * front_branch / rear_branch 决定。注意：当某条 2R 子链（O-A-B 或 D-C-B）
   * 接近完全伸直时（|OB| → l1+l2、|DB| → l4+l3），装配分支会翻转，固定分支
   * 只适用于工作空间的一侧。
   *
   * @param q_seed 非空时忽略 branch 参数，改为在 4 组解中取“离 q_seed 最近”的
   *               一组（角度按 2π 归一），用于连续轨迹/闭环里保持装配一致
   */
  bool inverseKinematics(
    const Eigen::Vector2d & foot, Eigen::Vector2d * q,
    const Eigen::Vector2d * q_seed = nullptr) const;

  /// 逆雅可比：足端速度 → 关节速度 q̇ = J⁻¹ ẋ
  bool jointVelocity(
    const Eigen::Vector2d & q, const Eigen::Vector2d & foot_velocity,
    Eigen::Vector2d * q_dot) const;

  /// 力矩映射：足端力 → 关节力矩 τ = Jᵀ F
  bool jointTorque(
    const Eigen::Vector2d & q, const Eigen::Vector2d & foot_force,
    Eigen::Vector2d * torque) const;
};

/// 虚拟弹簧-阻尼参数（逐轴：x 前后 / y 上下）
struct SpringDamperParams
{
  Eigen::Vector2d kp{60.0, 300.0};         ///< 刚度 [N/m]；y 轴需能撑住整机重量
  Eigen::Vector2d kd{3.0, 12.0};           ///< 阻尼 [N·s/m]
  Eigen::Vector2d force_max{40.0, 150.0};  ///< 足端力限幅 [N]，<=0 表示该轴不限幅
};

/**
 * @brief 足端弹簧-阻尼力 F = Kp·(x_d − x) + Kd·(ẋ_d − ẋ)，并按 force_max 逐轴限幅
 * @param target        期望足端位置 x_d
 * @param target_velocity 期望足端速度 ẋ_d
 * @param current       当前足端位置 x（正运动学）
 * @param current_velocity 当前足端速度 ẋ（J·q̇）
 */
Eigen::Vector2d springDamperForce(
  const SpringDamperParams & params,
  const Eigen::Vector2d & target, const Eigen::Vector2d & target_velocity,
  const Eigen::Vector2d & current, const Eigen::Vector2d & current_velocity);

/// 单腿一次 VMC 计算的中间量与结果
struct VmcLegState
{
  Eigen::Vector2d joint_position{Eigen::Vector2d::Zero()};   ///< q = [α, β]
  Eigen::Vector2d joint_velocity{Eigen::Vector2d::Zero()};   ///< q̇
  Eigen::Vector2d foot_position{Eigen::Vector2d::Zero()};    ///< x
  Eigen::Vector2d foot_velocity{Eigen::Vector2d::Zero()};    ///< ẋ = J q̇
  Eigen::Vector2d foot_force{Eigen::Vector2d::Zero()};       ///< F
  Eigen::Vector2d joint_torque{Eigen::Vector2d::Zero()};     ///< τ = Jᵀ F
  Eigen::Matrix2d jacobian{Eigen::Matrix2d::Zero()};         ///< J(q)
};

/// 一条腿的完整 VMC 链路：FK → 足端速度 → PD → Jᵀ；不可装配/奇异时返回 false
bool computeVmcLeg(
  const FiveBarGeometry & geometry, const SpringDamperParams & params,
  const Eigen::Vector2d & q, const Eigen::Vector2d & q_dot,
  const Eigen::Vector2d & target, const Eigen::Vector2d & target_velocity,
  VmcLegState * out);

}  // namespace leg_calc

#endif  // LEG_CALC__FOOT_VMC_HPP_
