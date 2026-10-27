#ifndef LEG_CALC__WHEEL_LEG_MODEL_HPP_
#define LEG_CALC__WHEEL_LEG_MODEL_HPP_

#include <Eigen/Dense>

namespace leg_calc
{

/**
 * @brief 状态向量索引
 *
 * x = [x, theta, x_dot, theta_dot]^T
 */
enum StateIndex : int
{
  kX = 0,        ///< 小车(轮轴中心)位移 [m]
  kTheta = 1,    ///< 摆杆偏角，绕轮轴、相对竖直向上方向为正 [rad]
  kXdot = 2,     ///< 小车速度 [m/s]
  kThetadot = 3, ///< 摆杆角速度 [rad/s]
  kStateDim = 4,
};

/**
 * @brief 控制输入的定义方式
 */
enum class InputMode
{
  kForce,       ///< u 为作用在轮轴(小车)上的等效力，单位 N
  kWheelTorque, ///< u 为折算到轮轴的驱动力矩，单位 N*m，F = u / r
};

/**
 * @brief 轮-摆(cart-pole)等效模型参数
 *
 * 双并腿轮足机器人被简化为“轮子=小车、腿+机身=摆杆”的欠驱动倒立摆：
 * 2 个自由度、1 个控制输入。
 */
struct ModelParams
{
  double cart_mass{1.0};       ///< M：轮子 + 底盘的等效平动质量 [kg]
  double pendulum_mass{2.0};   ///< m：摆杆(腿 + 机身)质量 [kg]
  double com_length{0.30};     ///< h：轮轴到摆杆质心的距离 [m]
  double com_inertia{0.02};    ///< I：摆杆绕自身质心的转动惯量 [kg*m^2]
  double gravity{9.81};        ///< g [m/s^2]
  double wheel_radius{0.065};  ///< r：轮子半径 [m]
  double wheel_inertia{0.0};   ///< Jw：轮子绕轮轴的等效转动惯量 [kg*m^2]
  InputMode input_mode{InputMode::kWheelTorque};
};

/**
 * @brief 双并腿轮足机器人的拉格朗日动力学模型
 *
 * ============================ 建模推导 ============================
 * 广义坐标 q = [q1, q2]^T = [x, theta]^T
 *
 * 1) 运动学：摆杆质心位置
 *      a = x + h*sin(theta)
 *      b =     h*cos(theta)
 *    质心速度
 *      a_dot = x_dot + h*theta_dot*cos(theta)
 *      b_dot =       - h*theta_dot*sin(theta)
 *
 * 2) 总动能
 *      T1 = 1/2 * M_eff * x_dot^2                     （小车平动，M_eff = M + Jw/r^2）
 *      T2 = 1/2 * m * (a_dot^2 + b_dot^2)
 *         = 1/2 * m * (x_dot^2 + 2*h*x_dot*theta_dot*cos(theta) + h^2*theta_dot^2)
 *      T3 = 1/2 * I * theta_dot^2                     （绕质心转动）
 *      T  = T1 + T2 + T3
 *
 * 3) 总势能（以轮轴为高度零点）
 *      V = m*g*h*cos(theta)
 *
 * 4) 拉格朗日方程
 *      d/dt(dL/dq_dot_i) - dL/dq_i = Q_i,   L = T - V
 *    对 x：
 *      (M_eff + m)*x_ddot + m*h*cos(theta)*theta_ddot - m*h*theta_dot^2*sin(theta) = F
 *    对 theta：
 *      m*h*cos(theta)*x_ddot + (I + m*h^2)*theta_ddot = m*g*h*sin(theta)
 *
 *    矩阵形式（本类的 massMatrix / biasVector 实现的就是这一形式）：
 *      D(theta) * q_ddot + h(q, q_dot) = Bq * u
 *      D(theta) = [ M_eff+m,        m*h*cos(theta) ]
 *                 [ m*h*cos(theta), I + m*h^2      ]
 *      h(q,q_dot) = [ -m*h*theta_dot^2*sin(theta) ]
 *                   [ -m*g*h*sin(theta)           ]
 *      Bq = [1/r, 0]^T（力矩输入）或 [1, 0]^T（力输入）
 *
 * 5) 直立平衡点小角度线性化：theta≈0 => sin(theta)≈theta, cos(theta)≈1, theta_dot^2≈0
 *      D0 = D(0) = [ M_eff+m, m*h     ]
 *                  [ m*h,     I + m*h^2 ]
 *      dh/dtheta|0 = [0, -m*g*h]^T,  dh/dtheta_dot|0 = 0,  dD/dtheta|0 = 0
 *      记 det = (M_eff + m)*I + M_eff*m*h^2
 *      A = [ 0            0      1 0 ]
 *          [ 0            0      0 1 ]
 *          [ 0  -m^2*g*h^2/det   0 0 ]
 *          [ 0  (M_eff+m)*m*g*h/det 0 0 ]
 *      B = D0^{-1} * Bq
 *
 * 注意：本模型是 cart-pole 等效模型，忽略了轮地打滑、腿关节主动力矩、
 * 力矩反作用于机身的耦合项等；若后续需要完整轮腿模型，可在同类接口上
 * 扩展为 q = [x, theta, ...] 的多自由度形式。
 * ==================================================================
 */
class WheelLegModel
{
public:
  using State = Eigen::Matrix<double, kStateDim, 1>;
  using MatA = Eigen::Matrix<double, kStateDim, kStateDim>;
  using MatB = Eigen::Matrix<double, kStateDim, 1>;

  WheelLegModel() = default;
  explicit WheelLegModel(const ModelParams & params);

  void setParams(const ModelParams & params);
  const ModelParams & params() const { return params_; }

  /// 折算后的等效平动质量 M_eff = M + Jw / r^2
  double effectiveCartMass() const;

  /// 质量矩阵 D(theta)
  Eigen::Matrix2d massMatrix(double theta) const;

  /// 科氏力 + 重力项 h(q, q_dot) = C(q, q_dot) * q_dot + G(q)
  Eigen::Vector2d biasVector(double theta, double theta_dot) const;

  /// 输入映射矩阵 Bq（广义力 = Bq * u）
  Eigen::Vector2d inputMatrix() const;

  /// 解算广义加速度 q_ddot
  Eigen::Vector2d jointAcceleration(double theta, double theta_dot, double u) const;

  /// 连续时间非线性动力学 x_dot = f(x, u)
  State dynamics(const State & state, double u) const;

  /// 四阶龙格库塔积分一步
  State integrateRK4(const State & state, double u, double dt) const;

  /**
   * @brief 直立平衡点(theta=0, theta_dot=0)处的解析线性化 x_dot = A x + B u
   *
   * 该模型对 u 是仿射的，因此 A、B 与平衡点的输入 u0 无关。
   */
  void linearize(MatA * A, MatB * B) const;

  /// 数值雅可比线性化（任意工作点），用于验证解析线性化结果
  void linearizeNumeric(const State & x0, double u0, double eps, MatA * A, MatB * B) const;

  /// 系统总机械能 T + V
  double totalEnergy(const State & state) const;

private:
  ModelParams params_{};
};

}  // namespace leg_calc

#endif  // LEG_CALC__WHEEL_LEG_MODEL_HPP_
