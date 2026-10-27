#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "leg_calc/lqr.hpp"
#include "leg_calc/quat_utils.hpp"
#include "leg_calc/wheel_leg_model.hpp"

namespace
{

leg_calc::ModelParams makeParams(leg_calc::InputMode mode)
{
  leg_calc::ModelParams params;
  params.cart_mass = 1.2;
  params.pendulum_mass = 2.5;
  params.com_length = 0.28;
  params.com_inertia = 0.05;
  params.gravity = 9.81;
  params.wheel_radius = 0.065;
  params.wheel_inertia = 0.0008;
  params.input_mode = mode;
  return params;
}

}  // namespace

// 解析线性化必须与数值雅可比一致（这是最容易出错的一步）
TEST(WheelLegModel, AnalyticLinearizationMatchesNumericJacobian)
{
  for (const auto mode : {leg_calc::InputMode::kForce, leg_calc::InputMode::kWheelTorque}) {
    leg_calc::WheelLegModel model(makeParams(mode));

    leg_calc::WheelLegModel::MatA A;
    leg_calc::WheelLegModel::MatB B;
    model.linearize(&A, &B);

    for (const double u0 : {0.0, 2.0, -5.0}) {
      leg_calc::WheelLegModel::MatA A_num;
      leg_calc::WheelLegModel::MatB B_num;
      model.linearizeNumeric(leg_calc::WheelLegModel::State::Zero(), u0, 1e-6, &A_num, &B_num);

      const double scale = std::max(1.0, std::max(A.cwiseAbs().maxCoeff(), B.cwiseAbs().maxCoeff()));
      EXPECT_LT((A - A_num).cwiseAbs().maxCoeff(), 1e-6 * scale);
      EXPECT_LT((B - B_num).cwiseAbs().maxCoeff(), 1e-6 * scale);
    }
  }
}

// A 的解析表达式应满足 det = (M+m)I + M*m*h^2 的结构（手推结果校核）
TEST(WheelLegModel, LinearizedPendulumBlockMatchesAnalyticFormula)
{
  const auto params = makeParams(leg_calc::InputMode::kForce);
  leg_calc::WheelLegModel model(params);

  leg_calc::WheelLegModel::MatA A;
  leg_calc::WheelLegModel::MatB B;
  model.linearize(&A, &B);

  const double M = model.effectiveCartMass();
  const double m = params.pendulum_mass;
  const double h = params.com_length;
  const double I = params.com_inertia;
  const double g = params.gravity;
  const double det = (M + m) * I + M * m * h * h;

  EXPECT_NEAR(A(leg_calc::kXdot, leg_calc::kTheta), -m * m * g * h * h / det, 1e-12);
  EXPECT_NEAR(A(leg_calc::kThetadot, leg_calc::kTheta), (M + m) * m * g * h / det, 1e-12);
  EXPECT_NEAR(A(leg_calc::kX, leg_calc::kXdot), 1.0, 1e-12);
  EXPECT_NEAR(A(leg_calc::kTheta, leg_calc::kThetadot), 1.0, 1e-12);
  EXPECT_GT(A(leg_calc::kThetadot, leg_calc::kTheta), 0.0);  // 倒立摆：正反馈 => 不稳定
}

// 质量矩阵应对称正定
TEST(WheelLegModel, MassMatrixIsSymmetricPositiveDefinite)
{
  leg_calc::WheelLegModel model(makeParams(leg_calc::InputMode::kWheelTorque));
  for (const double theta : {0.0, 0.3, -0.9, 1.4}) {
    const Eigen::Matrix2d D = model.massMatrix(theta);
    EXPECT_NEAR(D(0, 1), D(1, 0), 1e-15);
    const Eigen::LDLT<Eigen::Matrix2d> ldlt(D);
    ASSERT_EQ(ldlt.info(), Eigen::Success);
    EXPECT_GT(ldlt.vectorD()(0), 0.0);
    EXPECT_GT(ldlt.vectorD()(1), 0.0);
  }
}

// 无外力时非线性系统的总机械能应守恒（验证动力学方程内部自洽）
TEST(WheelLegModel, EnergyIsConservedWithoutInput)
{
  leg_calc::WheelLegModel model(makeParams(leg_calc::InputMode::kForce));

  leg_calc::WheelLegModel::State state;
  state << 0.3, 0.35, 0.2, 0.8;
  const double energy0 = model.totalEnergy(state);

  const double dt = 1e-4;
  for (int i = 0; i < 30000; ++i) {  // 3 s
    state = model.integrateRK4(state, 0.0, dt);
  }
  const double energy1 = model.totalEnergy(state);

  EXPECT_NEAR(energy1, energy0, 1e-6 * std::abs(energy0));
}

// LQR：Riccati 残差、闭环稳定性
TEST(LqrSolver, ProducesStableClosedLoop)
{
  leg_calc::WheelLegModel model(makeParams(leg_calc::InputMode::kWheelTorque));

  leg_calc::WheelLegModel::MatA A;
  leg_calc::WheelLegModel::MatB B;
  model.linearize(&A, &B);

  Eigen::MatrixXd Q = Eigen::Vector4d(10.0, 500.0, 1.0, 1.0).asDiagonal();
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

  const leg_calc::LqrSolution sol = leg_calc::solveContinuousLqr(A, B, Q, R);
  ASSERT_TRUE(sol.valid);
  ASSERT_TRUE(sol.isStable());
  EXPECT_LT(sol.riccati_residual, 1e-8);
  EXPECT_EQ(sol.gain.rows(), 1);
  EXPECT_EQ(sol.gain.cols(), 4);
}

// 用非线性模型闭环仿真，验证线性 LQR 在原系统上确实能把摆杆扶起来
TEST(LqrSolver, StabilizesNonlinearModel)
{
  leg_calc::WheelLegModel model(makeParams(leg_calc::InputMode::kWheelTorque));

  leg_calc::WheelLegModel::MatA A;
  leg_calc::WheelLegModel::MatB B;
  model.linearize(&A, &B);

  Eigen::MatrixXd Q = Eigen::Vector4d(10.0, 500.0, 1.0, 1.0).asDiagonal();
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);
  const leg_calc::LqrSolution sol = leg_calc::solveContinuousLqr(A, B, Q, R);
  ASSERT_TRUE(sol.valid && sol.isStable());

  leg_calc::WheelLegModel::State state;
  state << 0.0, 0.25, 0.0, 0.0;

  const double dt = 1e-3;
  for (int i = 0; i < 10000; ++i) {  // 10 s
    const double u = -(sol.gain * state)(0, 0);
    state = model.integrateRK4(state, u, dt);
  }

  EXPECT_LT(std::abs(state(leg_calc::kTheta)), 1e-3);
  EXPECT_LT(std::abs(state(leg_calc::kXdot)), 1e-2);
  EXPECT_LT(std::abs(state(leg_calc::kThetadot)), 1e-2);
}

// 四元数 -> 欧拉角（theta 取 pitch）转换正确性
TEST(QuatUtils, ConvertsKnownAttitudes)
{
  constexpr double kPi = 3.14159265358979323846;

  // 单位四元数：零姿态
  Eigen::Vector3d rpy = leg_calc::quaternionToRpy(1.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(rpy.norm(), 0.0, 1e-12);

  // 绕 y 轴 +30 deg（俯仰）：q = (cos15, 0, sin15, 0)
  const double half = kPi / 12.0;
  rpy = leg_calc::quaternionToRpy(std::cos(half), 0.0, std::sin(half), 0.0);
  EXPECT_NEAR(rpy(0), 0.0, 1e-12);
  EXPECT_NEAR(rpy(1), kPi / 6.0, 1e-12);
  EXPECT_NEAR(rpy(2), 0.0, 1e-12);

  // 绕 z 轴 +90 deg（偏航）：q = (cos45, 0, 0, sin45)
  rpy = leg_calc::quaternionToRpy(std::cos(kPi / 4.0), 0.0, 0.0, std::sin(kPi / 4.0));
  EXPECT_NEAR(rpy(2), kPi / 2.0, 1e-12);

  // 未归一化四元数也应得到同样结果
  rpy = leg_calc::quaternionToRpy(2.0 * std::cos(half), 0.0, 2.0 * std::sin(half), 0.0);
  EXPECT_NEAR(rpy(1), kPi / 6.0, 1e-12);
}

// 关节角 -> 质心高度
TEST(LegGeometry, ComputesComHeightFromJoints)
{
  leg_calc::LegGeometry geometry;
  geometry.thigh = 0.15;
  geometry.shank = 0.15;
  geometry.com_offset = 0.10;

  // 腿完全伸直：h = l1 + l2 + offset
  EXPECT_NEAR(geometry.comHeight(0.0, 0.0), 0.40, 1e-12);

  // 髋关节 90 度：大腿水平，只剩小腿竖直
  EXPECT_NEAR(geometry.comHeight(M_PI / 2.0, 0.0), 0.10, 1e-12);

  // 双腿平均
  std::array<double, 4> joints{0.0, 0.0, 0.2, 0.0};
  EXPECT_NEAR(
    geometry.comHeight(joints),
    0.5 * (geometry.comHeight(0.0, 0.0) + geometry.comHeight(0.2, 0.0)), 1e-12);
}

// 二次多项式模式：K(h) 必须与系数严格对应，且 h 截断生效
TEST(GainSchedule, QuadraticModeEvaluatesPolynomial)
{
  leg_calc::GainSchedule schedule(makeParams(leg_calc::InputMode::kWheelTorque),
    leg_calc::GainMode::kQuadratic);

  Eigen::Matrix<double, 4, 3> coeffs;
  coeffs << 1.0, 2.0, 3.0,
    0.0, -1.0, 0.5,
    0.1, 0.2, 0.3,
    -0.5, 0.4, -0.2;
  schedule.setQuadraticCoefficients(coeffs);
  schedule.setHeightLimits(0.10, 0.60);
  ASSERT_TRUE(schedule.hasQuadraticCoefficients());

  const double h = 0.33;
  const auto gain = schedule.gainAt(h);
  for (int j = 0; j < 4; ++j) {
    EXPECT_NEAR(gain(0, j), coeffs(j, 0) * h * h + coeffs(j, 1) * h + coeffs(j, 2), 1e-12);
  }

  // 超出区间按 h_min 截断
  const auto gain_low = schedule.gainAt(0.0);
  EXPECT_NEAR(gain_low(0, 0), coeffs(0, 0) * 0.01 + coeffs(0, 1) * 0.10 + coeffs(0, 2), 1e-12);
}

// 在线 LQR 模式：结果应与“按该 h 重建模型直接求解”完全一致
TEST(GainSchedule, OnlineLqrMatchesDirectSolve)
{
  const auto params = makeParams(leg_calc::InputMode::kWheelTorque);
  const Eigen::MatrixXd Q = Eigen::Vector4d(10.0, 500.0, 1.0, 1.0).asDiagonal();
  const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

  leg_calc::GainSchedule schedule(params, leg_calc::GainMode::kOnlineLqr);
  schedule.setWeights(Q, R);
  schedule.setHeightLimits(0.10, 0.60);

  for (const double h : {0.20, 0.28, 0.36}) {
    const auto gain = schedule.gainAt(h);

    leg_calc::ModelParams scaled = params;
    scaled.com_length = h;
    leg_calc::WheelLegModel model(scaled);
    leg_calc::WheelLegModel::MatA A;
    leg_calc::WheelLegModel::MatB B;
    model.linearize(&A, &B);
    const leg_calc::LqrSolution solution = leg_calc::solveContinuousLqr(A, B, Q, R);
    ASSERT_TRUE(solution.valid);

    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(gain(0, j), solution.gain(0, j), 1e-9);
    }
  }
}

// 系数全零时应自动退回在线 LQR（而不是输出零增益）
TEST(GainSchedule, FallsBackToOnlineLqrWhenCoefficientsEmpty)
{
  leg_calc::GainSchedule schedule(makeParams(leg_calc::InputMode::kWheelTorque),
    leg_calc::GainMode::kQuadratic);
  schedule.setWeights(
    Eigen::Vector4d(10.0, 500.0, 1.0, 1.0).asDiagonal(),
    Eigen::MatrixXd::Identity(1, 1));
  schedule.setQuadraticCoefficients(Eigen::Matrix<double, 4, 3>::Zero());
  ASSERT_FALSE(schedule.hasQuadraticCoefficients());

  const auto gain = schedule.gainAt(0.28);
  EXPECT_GT(std::abs(gain(0, 1)), 1.0);  // theta 通道增益应显著非零
}
