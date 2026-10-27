#ifndef LEG_CALC__DIFF_DRIVE_HPP_
#define LEG_CALC__DIFF_DRIVE_HPP_

// 差速轮底盘运动学（正解 / 反解）
//
//              y（车头）
//              ↑
//       ω_L ──┤├── ω_R        左轮在 +y 侧，右轮在 −y 侧，轮距 L
//              └──► x
//
// 正解（轮速 → 底盘）：
//   v = (ω_L + ω_R)·r / 2
//   ω = (ω_R − ω_L)·r / L
//
// 反解（底盘 → 轮速）：
//   ω_L = (v − L/2·ω) / r
//   ω_R = (v + L/2·ω) / r
//
// 符号约定（与 odom、/joint_states 里 wheel_*_joint.velocity 一致）：
//   * ω > 0 → 车体左转（逆时针，yaw 增大）：右轮快、左轮慢
//   * ω < 0 → 车体右转（顺时针，yaw 减小）
//   * 原地左转：v = 0、ω > 0 → ω_L < 0、ω_R > 0
//   * ω_L = ω_R → 直线前进（>0 前进，<0 后退）
//
// 单位：长度 m，线速度 m/s，角速度 rad/s。

namespace leg_calc
{

/// 差速底盘参数
struct DiffDriveParams
{
  double wheel_separation{0.20};  ///< L：左右轮间距 [m]
  double wheel_radius{0.065};     ///< r：轮半径 [m]
  double max_wheel_speed{20.0};   ///< 单轮角速度限幅 [rad/s]，<=0 表示不限
  double max_linear{1.0};         ///< 底盘线速度限幅 [m/s]，<=0 表示不限
  double max_angular{3.0};        ///< 底盘角速度限幅 [rad/s]，<=0 表示不限

  /// L 与 r 必须为正
  bool isValid() const {return wheel_separation > 1e-9 && wheel_radius > 1e-9;}
};

/// 一次解算的结果：底盘量 + 左右轮量
struct DiffDriveState
{
  double linear{0.0};        ///< v [m/s]
  double angular{0.0};       ///< ω [rad/s]
  double left_linear{0.0};   ///< 左轮线速度 [m/s]
  double right_linear{0.0};  ///< 右轮线速度 [m/s]
  double left_speed{0.0};    ///< 左轮角速度 [rad/s]
  double right_speed{0.0};   ///< 右轮角速度 [rad/s]
  bool saturated{false};     ///< 是否触发了限幅
};

/// 正解：左右轮角速度 [rad/s] → 底盘 v、ω（不做限幅）
DiffDriveState bodyFromWheels(
  const DiffDriveParams & params, double left_speed, double right_speed);

/**
 * @brief 反解：底盘 v、ω → 左右轮角速度 [rad/s]
 *
 * 限幅顺序：
 *   1. 先把 v、ω 截到 max_linear / max_angular（独立限幅，保持输入意图）；
 *   2. 再算轮速；若某轮超 max_wheel_speed，则**左右轮按同一比例缩放**，
 *      这样不会改变转向半径（代价是 v、ω 同比例变小）。
 *
 * 参数非法（L<=0 或 r<=0）时返回全零。
 */
DiffDriveState wheelsFromBody(
  const DiffDriveParams & params, double linear, double angular);

}  // namespace leg_calc

#endif  // LEG_CALC__DIFF_DRIVE_HPP_
