#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "leg_calc/foot_vmc.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

leg_calc::FiveBarGeometry makeGeometry()
{
  // 默认几何：l1 = l4 = 0.10，l2 = l3 = 0.20，l5 = 0.10
  return leg_calc::FiveBarGeometry{};
}

}  // namespace

TEST(FiveBarGeometry, ValidatesLinkLengths)
{
  leg_calc::FiveBarGeometry geo = makeGeometry();
  EXPECT_TRUE(geo.isValid());

  geo.l2 = 0.01;  // 前连杆太短，α=β=90° 时 A、C 间距 0.10 接不上
  EXPECT_FALSE(geo.isValid());

  geo = makeGeometry();
  geo.l4 = -0.1;
  EXPECT_FALSE(geo.isValid());
}

// 图中装配：α = β = 90° 时两摆臂竖直向下，足端在 O-D 中点正下方
TEST(FiveBarGeometry, NeutralPoseMatchesHandCalculation)
{
  const leg_calc::FiveBarGeometry geo = makeGeometry();
  Eigen::Vector2d foot;
  ASSERT_TRUE(geo.footPosition(Eigen::Vector2d(kPi / 2.0, kPi / 2.0), &foot));

  const double expected_y =
    geo.l1 + std::sqrt(geo.l2 * geo.l2 - 0.25 * geo.l5 * geo.l5);
  EXPECT_NEAR(foot.x(), 0.5 * geo.l5, 1e-12);
  EXPECT_NEAR(foot.y(), expected_y, 1e-12);
  EXPECT_NEAR(foot.y(), 0.293649, 1e-6);

  // 前肘在 O 正下方、后肘在 D 正下方
  EXPECT_NEAR(geo.frontElbow(kPi / 2.0).x(), 0.0, 1e-12);
  EXPECT_NEAR(geo.frontElbow(kPi / 2.0).y(), geo.l1, 1e-12);
  EXPECT_NEAR(geo.rearElbow(kPi / 2.0).x(), geo.l5, 1e-12);
  EXPECT_NEAR(geo.rearElbow(kPi / 2.0).y(), geo.l4, 1e-12);
}

TEST(FiveBarGeometry, RejectsUnreachableAssembly)
{
  leg_calc::FiveBarGeometry geo = makeGeometry();
  geo.l2 = 0.04;
  geo.l3 = 0.04;  // 两连杆加起来比 A、C 间距还短

  Eigen::Vector2d foot;
  EXPECT_FALSE(geo.footPosition(Eigen::Vector2d(0.0, 0.0), &foot));
}

// 解析雅可比必须与数值差分一致（最容易写错的一步）
TEST(FiveBarGeometry, JacobianMatchesNumericDifferentiation)
{
  const leg_calc::FiveBarGeometry geo = makeGeometry();
  const std::vector<Eigen::Vector2d> poses = {
    {kPi / 2.0, kPi / 2.0},
    {1.1, 1.4},
    {1.9, 1.0},
    {0.9, 2.0},
  };

  for (const auto & q : poses) {
    Eigen::Matrix2d jac;
    ASSERT_TRUE(geo.jacobian(q, &jac)) << "q = " << q.transpose();

    const double eps = 1e-6;
    Eigen::Matrix2d numeric;
    for (int i = 0; i < 2; ++i) {
      Eigen::Vector2d q_plus = q;
      Eigen::Vector2d q_minus = q;
      q_plus(i) += eps;
      q_minus(i) -= eps;

      Eigen::Vector2d foot_plus;
      Eigen::Vector2d foot_minus;
      ASSERT_TRUE(geo.footPosition(q_plus, &foot_plus));
      ASSERT_TRUE(geo.footPosition(q_minus, &foot_minus));
      numeric.col(i) = (foot_plus - foot_minus) / (2.0 * eps);
    }
    EXPECT_LT((jac - numeric).cwiseAbs().maxCoeff(), 1e-5)
      << "q = " << q.transpose() << "\nJ = \n" << jac << "\nnumeric = \n" << numeric;
  }
}

// 固定装配分支：只在中位附近（子链未接近完全伸直）成立
TEST(FiveBarGeometry, InverseKinematicsRoundTripNearNeutral)
{
  const leg_calc::FiveBarGeometry geo = makeGeometry();
  for (double alpha_deg = 86.0; alpha_deg <= 96.0; alpha_deg += 5.0) {
    for (double beta_deg = 86.0; beta_deg <= 96.0; beta_deg += 5.0) {
      const Eigen::Vector2d q(alpha_deg * kPi / 180.0, beta_deg * kPi / 180.0);

      Eigen::Vector2d foot;
      ASSERT_TRUE(geo.footPosition(q, &foot));

      Eigen::Vector2d q_solved;
      ASSERT_TRUE(geo.inverseKinematics(foot, &q_solved)) << "α=" << alpha_deg <<
        " β=" << beta_deg;
      EXPECT_NEAR(q_solved(0), q(0), 1e-9);
      EXPECT_NEAR(q_solved(1), q(1), 1e-9);
    }
  }
}

// 连续轨迹：用实测关节角作种子，逆解应稳定跟随（大范围扫过工作空间）。
// 某条 2R 子链接近完全伸直时两组装配解会重合，此时只要求足端位置复原。
TEST(FiveBarGeometry, InverseKinematicsFollowsContinuousPath)
{
  const leg_calc::FiveBarGeometry geo = makeGeometry();
  Eigen::Vector2d seed(kPi / 2.0, kPi / 2.0);

  for (double t = 0.0; t <= 1.0001; t += 0.01) {
    const Eigen::Vector2d q(
      kPi / 2.0 + 0.6 * std::sin(3.0 * t),
      kPi / 2.0 - 0.5 * std::cos(2.0 * t));

    Eigen::Vector2d foot;
    ASSERT_TRUE(geo.footPosition(q, &foot));

    Eigen::Vector2d q_solved;
    ASSERT_TRUE(geo.inverseKinematics(foot, &q_solved, &seed)) << "t = " << t;

    Eigen::Vector2d foot_back;
    ASSERT_TRUE(geo.footPosition(q_solved, &foot_back));
    EXPECT_LT((foot_back - foot).norm(), 1e-4) << "t = " << t;

    const double front_chain = foot.norm();                    // |OB|
    const double rear_chain = (foot - geo.hipFar()).norm();    // |DB|
    const bool unique = std::abs(front_chain - (geo.l1 + geo.l2)) > 0.005 &&
      std::abs(rear_chain - (geo.l3 + geo.l4)) > 0.005;
    if (unique) {
      EXPECT_NEAR(q_solved(0), q(0), 1e-9) << "t = " << t;
      EXPECT_NEAR(q_solved(1), q(1), 1e-9) << "t = " << t;
    }
    seed = q;
  }
}

TEST(FiveBarGeometry, InverseKinematicsRejectsFarTarget)
{
  const leg_calc::FiveBarGeometry geo = makeGeometry();
  Eigen::Vector2d q;
  EXPECT_FALSE(geo.inverseKinematics(Eigen::Vector2d(0.05, 1.0), &q));
}

TEST(FiveBarGeometry, JointVelocityRoundTrip)
{
  const leg_calc::FiveBarGeometry geo = makeGeometry();
  const Eigen::Vector2d q(1.3, 1.6);
  const Eigen::Vector2d q_dot(0.7, -0.4);

  Eigen::Matrix2d jac;
  ASSERT_TRUE(geo.jacobian(q, &jac));
  const Eigen::Vector2d foot_velocity = jac * q_dot;

  Eigen::Vector2d solved;
  ASSERT_TRUE(geo.jointVelocity(q, foot_velocity, &solved));
  EXPECT_LT((solved - q_dot).cwiseAbs().maxCoeff(), 1e-12);
}

// 虚功一致性：F·ẋ = τ·q̇，即 τ = JᵀF 的等价表述
TEST(FiveBarGeometry, TorqueMappingSatisfiesVirtualWork)
{
  const leg_calc::FiveBarGeometry geo = makeGeometry();
  const Eigen::Vector2d q(1.2, 1.7);
  const Eigen::Vector2d q_dot(0.5, -0.9);
  const Eigen::Vector2d force(3.0, -12.0);

  Eigen::Matrix2d jac;
  ASSERT_TRUE(geo.jacobian(q, &jac));
  Eigen::Vector2d torque;
  ASSERT_TRUE(geo.jointTorque(q, force, &torque));

  const double foot_power = force.dot(jac * q_dot);
  const double joint_power = torque.dot(q_dot);
  EXPECT_NEAR(foot_power, joint_power, 1e-9);
}

TEST(SpringDamper, ZeroErrorGivesZeroForce)
{
  const leg_calc::SpringDamperParams params;
  const Eigen::Vector2d x(0.05, 0.30);
  const Eigen::Vector2d zero = Eigen::Vector2d::Zero();

  const Eigen::Vector2d force =
    leg_calc::springDamperForce(params, x, zero, x, zero);
  EXPECT_LT(force.cwiseAbs().maxCoeff(), 1e-12);
}

// 足端比目标高（y 更小）时，弹簧应把足端往下推（F_y > 0）
TEST(SpringDamper, RestoringForcePointsToTarget)
{
  const leg_calc::SpringDamperParams params;
  const Eigen::Vector2d zero = Eigen::Vector2d::Zero();
  const Eigen::Vector2d force = leg_calc::springDamperForce(
    params, Eigen::Vector2d(0.05, 0.30), zero, Eigen::Vector2d(0.05, 0.24), zero);

  EXPECT_NEAR(force.x(), 0.0, 1e-12);
  EXPECT_GT(force.y(), 0.0);
  EXPECT_NEAR(force.y(), params.kp(1) * 0.06, 1e-9);
}

TEST(SpringDamper, DampingActsOnVelocityError)
{
  const leg_calc::SpringDamperParams params;
  const Eigen::Vector2d x(0.05, 0.30);
  const Eigen::Vector2d zero = Eigen::Vector2d::Zero();

  // 足端正以 +y 方向（向下）运动，超过目标速度 → 阻尼力向上（F_y < 0）
  const Eigen::Vector2d force = leg_calc::springDamperForce(
    params, x, zero, x, Eigen::Vector2d(0.0, 1.0));
  EXPECT_NEAR(force.y(), -params.kd(1), 1e-9);
}

TEST(SpringDamper, ClampsForcePerAxis)
{
  leg_calc::SpringDamperParams params;
  params.kp = Eigen::Vector2d(1000.0, 1000.0);
  params.force_max = Eigen::Vector2d(10.0, 20.0);

  const Eigen::Vector2d force = leg_calc::springDamperForce(
    params, Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d::Zero(),
    Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());

  EXPECT_NEAR(force.x(), 10.0, 1e-12);
  EXPECT_NEAR(force.y(), 20.0, 1e-12);

  params.force_max = Eigen::Vector2d(0.0, -1.0);  // <= 0 表示不限幅
  const Eigen::Vector2d unlimited = leg_calc::springDamperForce(
    params, Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d::Zero(),
    Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());
  EXPECT_NEAR(unlimited.x(), 1000.0, 1e-9);
  EXPECT_NEAR(unlimited.y(), 1000.0, 1e-9);
}

// 完整链路：足端被压缩（实际 y < 目标 y）→ 力向下 → 关节力矩有限且符号一致
TEST(Vmc, ComputesFullChain)
{
  const leg_calc::FiveBarGeometry geo = makeGeometry();
  leg_calc::SpringDamperParams params;
  params.kp = Eigen::Vector2d(200.0, 400.0);
  params.kd = Eigen::Vector2d(0.0, 0.0);
  params.force_max = Eigen::Vector2d(0.0, 0.0);

  const Eigen::Vector2d q(kPi / 2.0, kPi / 2.0);
  const Eigen::Vector2d zero = Eigen::Vector2d::Zero();
  const Eigen::Vector2d target(0.05, 0.35);  // 比当前足端更低 → 弹簧往下推

  leg_calc::VmcLegState state;
  ASSERT_TRUE(leg_calc::computeVmcLeg(geo, params, q, zero, target, zero, &state));

  EXPECT_NEAR(state.foot_position.y(), 0.293649, 1e-6);
  EXPECT_GT(state.foot_force.y(), 0.0);
  EXPECT_NEAR(state.foot_force.y(), 400.0 * (target.y() - state.foot_position.y()), 1e-9);

  Eigen::Vector2d expected_torque;
  ASSERT_TRUE(geo.jointTorque(q, state.foot_force, &expected_torque));
  EXPECT_LT((state.joint_torque - expected_torque).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_GT(state.joint_torque.cwiseAbs().maxCoeff(), 0.0);
}

TEST(Vmc, FailsOnSingularPose)
{
  leg_calc::FiveBarGeometry geo = makeGeometry();
  geo.l2 = 0.05;
  geo.l3 = 0.05;
  geo.foot_branch = -1;

  leg_calc::VmcLegState state;
  // α=β=90° 时 A、C 间距 0.10 = l2 + l3，两连杆共线 → 雅可比奇异
  EXPECT_FALSE(
    leg_calc::computeVmcLeg(
      geo, leg_calc::SpringDamperParams{}, Eigen::Vector2d(kPi / 2.0, kPi / 2.0),
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), &state));
}
