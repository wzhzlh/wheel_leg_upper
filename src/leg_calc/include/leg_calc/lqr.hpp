#ifndef LEG_CALC__LQR_HPP_
#define LEG_CALC__LQR_HPP_

// LQR 全链路：Riccati 求解 → 腿部几何 → 随质心高度调度的反馈增益。

#include <Eigen/Dense>

#include <array>

#include "leg_calc/wheel_leg_model.hpp"

namespace leg_calc
{

/**
 * @brief 连续时间 LQR 求解结果
 *
 * 控制律：u = -K (x - x_ref)
 * 代价函数：J = ∫ (x^T Q x + u^T R u) dt
 */
struct LqrSolution
{
  Eigen::MatrixXd gain;      ///< 最优反馈增益 K = R^{-1} B^T P
  Eigen::MatrixXd riccati;   ///< 代数 Riccati 方程的解 P
  Eigen::VectorXd closed_loop_eigenvalues_real;  ///< A - B*K 特征值实部
  Eigen::VectorXd closed_loop_eigenvalues_imag;  ///< A - B*K 特征值虚部
  double riccati_residual{0.0};                  ///< ||A^T P + P A - P B R^-1 B^T P + Q||
  bool valid{false};                             ///< 求解是否成功（P 有效且正定）

  /// 闭环是否渐近稳定
  bool isStable() const;
};

/**
 * @brief 求解连续时间代数 Riccati 方程（CARE）
 *
 * A^T P + P A - P B R^{-1} B^T P + Q = 0
 *
 * 方法：构造哈密顿矩阵
 *   H = [ A          -B R^{-1} B^T ]
 *       [ -Q         -A^T           ]
 * 对 H 做特征分解，取实部为负的 n 个稳定特征向量 [V1; V2]，则
 *   P = V2 * V1^{-1}
 *
 * @param A 系统矩阵 (n x n)
 * @param B 输入矩阵 (n x m)
 * @param Q 状态权重，半正定对称
 * @param R 输入权重，正定对称
 */
LqrSolution solveContinuousLqr(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R);

/**
 * @brief 腿部几何：由四个关节角估算质心高度 h
 *
 * 每条腿 2 个关节（髋 q1、膝 q2），轮轴到髋关节的距离为
 *   d(q1, q2) = l1 * cos(q1) + l2 * cos(q1 + q2)
 * 质心（机身）相对髋关节还有一段固定偏置，于是
 *   h(q1, q2) = l1 * cos(q1) + l2 * cos(q1 + q2) + com_offset
 *
 * 真实机构若为平行四边形/连杆式，只需改写 LegGeometry::comHeight 即可，
 * 上层控制器只依赖“关节角 -> h”这一层接口。
 */
struct LegGeometry
{
  double thigh{0.15};       ///< l1：髋关节到大腿末端（膝关节）长度 [m]
  double shank{0.15};       ///< l2：膝关节到轮轴长度 [m]
  double com_offset{0.10};  ///< 髋关节到质心的距离，质心在髋上方为正 [m]

  /// 单腿质心高度
  double comHeight(double hip, double knee) const;

  /// 双腿平均质心高度，joints = [左髋, 左膝, 右髋, 右膝]
  double comHeight(const std::array<double, 4> & joints) const;
};

/// 反馈增益 K 的获取方式
enum class GainMode
{
  kQuadratic,  ///< K_j(h) = c2*h^2 + c1*h + c0，系数由外部拟合后写入（快，适合实时）
  kOnlineLqr,  ///< 每个新的 h 重新求解一次 LQR（无需拟合，便于对照拟合结果）
};

/**
 * @brief 反馈增益调度器：K = K(h)
 *
 * 用途：腿的四个关节改变机身高度 h，而 h 变化会改变倒立摆的质量/惯量分布，
 * 因此 LQR 增益需要随 h 调度。两条路径：
 *   1. 二次多项式（默认）：K_j(h) = c2*h^2 + c1*h + c0，系数矩阵 4x3 由外部提供，
 *      可用 offline 扫 h 求 LQR 后拟合得到；
 *   2. 在线 LQR：直接按 h 重建模型并求解 Riccati 方程，用于生成拟合数据/校验。
 *
 * 所有参数都可通过 setter 在运行时修改，方便整定。
 */
class GainSchedule
{
public:
  using Gain = Eigen::Matrix<double, 1, 4>;

  GainSchedule() = default;
  GainSchedule(const ModelParams & params, GainMode mode);

  void setModelParams(const ModelParams & params);
  const ModelParams & params() const {return params_;}

  void setMode(GainMode mode);
  GainMode mode() const {return mode_;}

  /// LQR 权重：Q(4x4) 与 R(1x1)
  void setWeights(const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R);

  /**
   * @brief 设定二次多项式系数
   * @param coeffs 4x3，行对应 [k_x, k_theta, k_x_dot, k_theta_dot]，
   *               列对应 [h^2, h, 1]
   */
  void setQuadraticCoefficients(const Eigen::Matrix<double, 4, 3> & coeffs);
  const Eigen::Matrix<double, 4, 3> & quadraticCoefficients() const {return coeffs_;}
  bool hasQuadraticCoefficients() const {return coeffs_valid_;}

  /// h 的合法区间，超出后截断，避免关节角异常时增益发散
  void setHeightLimits(double h_min, double h_max);

  /// 依据当前质心高度返回反馈增益（二次多项式优先，否则在线求解）
  Gain gainAt(double com_height);

  /// 强制在给定 h 处求一次 LQR（忽略多项式系数与缓存）
  Gain solveLqrAt(double com_height);

  /// 上一次在线求解是否成功
  bool lastSolveOk() const {return last_solve_ok_;}

private:
  ModelParams params_{};
  GainMode mode_{GainMode::kQuadratic};

  Eigen::MatrixXd Q_{Eigen::Matrix4d::Identity()};
  Eigen::MatrixXd R_{Eigen::MatrixXd::Identity(1, 1)};

  Eigen::Matrix<double, 4, 3> coeffs_{Eigen::Matrix<double, 4, 3>::Zero()};
  bool coeffs_valid_{false};

  double h_min_{0.05};
  double h_max_{1.00};

  // 在线 LQR 的缓存（h 变化超过阈值才重算）
  double cached_h_{-1.0};
  Gain cached_gain_{Gain::Zero()};
  bool cache_valid_{false};
  bool last_solve_ok_{false};
};

}  // namespace leg_calc

#endif  // LEG_CALC__LQR_HPP_
