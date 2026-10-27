#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "leg_calc/diff_drive.hpp"

namespace
{

// 默认参数：L = 0.20 m，r = 0.065 m，限幅 20 rad/s / 1 m/s / 3 rad/s
leg_calc::DiffDriveParams makeParams()
{
  return leg_calc::DiffDriveParams{};
}

}  // namespace

TEST(DiffDrive, ForwardStraight)
{
  const auto params = makeParams();
  const auto state = leg_calc::bodyFromWheels(params, 5.0, 5.0);

  EXPECT_NEAR(state.linear, 5.0 * params.wheel_radius, 1e-12);
  EXPECT_NEAR(state.angular, 0.0, 1e-12);
}

// ω > 0 为左转：左轮倒转、右轮正转
TEST(DiffDrive, ForwardSpinInPlaceLeft)
{
  const auto params = makeParams();
  const auto state = leg_calc::bodyFromWheels(params, -5.0, 5.0);

  EXPECT_NEAR(state.linear, 0.0, 1e-12);
  EXPECT_NEAR(state.angular, 10.0 * params.wheel_radius / params.wheel_separation, 1e-12);
  EXPECT_GT(state.angular, 0.0);
}

// ω < 0 为右转
TEST(DiffDrive, ForwardSpinInPlaceRight)
{
  const auto params = makeParams();
  const auto state = leg_calc::bodyFromWheels(params, 5.0, -5.0);
  EXPECT_LT(state.angular, 0.0);
}

TEST(DiffDrive, InverseStraight)
{
  const auto params = makeParams();
  const auto state = leg_calc::wheelsFromBody(params, 0.5, 0.0);

  EXPECT_NEAR(state.left_speed, 0.5 / params.wheel_radius, 1e-12);
  EXPECT_NEAR(state.right_speed, 0.5 / params.wheel_radius, 1e-12);
  EXPECT_NEAR(state.left_linear, 0.5, 1e-12);
  EXPECT_NEAR(state.right_linear, 0.5, 1e-12);
  EXPECT_FALSE(state.saturated);
}

TEST(DiffDrive, InverseSpinInPlaceLeft)
{
  const auto params = makeParams();
  const auto state = leg_calc::wheelsFromBody(params, 0.0, 1.0);

  const double expected = 0.5 * params.wheel_separation / params.wheel_radius;
  EXPECT_NEAR(state.left_speed, -expected, 1e-12);
  EXPECT_NEAR(state.right_speed, expected, 1e-12);
  EXPECT_LT(state.left_speed, 0.0);
}

// 反解再正解应回到原来的 (v, ω)
TEST(DiffDrive, InverseThenForwardRoundTrip)
{
  const auto params = makeParams();
  const std::vector<std::pair<double, double>> commands = {
    {0.30, 0.0},
    {0.0, 1.20},
    {-0.20, -0.80},
    {0.50, 2.50},
  };

  for (const auto & cmd : commands) {
    const auto wheels = leg_calc::wheelsFromBody(params, cmd.first, cmd.second);
    ASSERT_FALSE(wheels.saturated);

    const auto body = leg_calc::bodyFromWheels(params, wheels.left_speed, wheels.right_speed);
    EXPECT_NEAR(body.linear, cmd.first, 1e-12);
    EXPECT_NEAR(body.angular, cmd.second, 1e-12);
  }
}

// 轮速超限时左右同比例缩放：转向半径（v/ω 比值）不变
TEST(DiffDrive, ClampsWheelSpeedPreservingRatio)
{
  auto params = makeParams();
  params.max_wheel_speed = 5.0;

  const auto state = leg_calc::wheelsFromBody(params, 1.0, 1.0);

  EXPECT_TRUE(state.saturated);
  EXPECT_NEAR(std::max(std::abs(state.left_speed), std::abs(state.right_speed)), 5.0, 1e-12);
  // v/ω 仍为 1.0
  EXPECT_NEAR(state.linear / state.angular, 1.0, 1e-12);
  EXPECT_NEAR(state.linear, state.angular, 1e-12);
}

// 车体量先独立限幅
TEST(DiffDrive, ClampsBodySpeeds)
{
  auto params = makeParams();
  params.max_linear = 0.5;
  params.max_angular = 0.0;  // 0 表示不限

  const auto state = leg_calc::wheelsFromBody(params, 2.0, 0.0);

  EXPECT_NEAR(state.linear, 0.5, 1e-12);
  EXPECT_TRUE(state.saturated);
  EXPECT_NEAR(state.left_speed, 0.5 / params.wheel_radius, 1e-12);
}

// 限幅值 <= 0 表示不限
TEST(DiffDrive, NonPositiveLimitsMeanUnlimited)
{
  auto params = makeParams();
  params.max_linear = 0.0;
  params.max_angular = 0.0;
  params.max_wheel_speed = 0.0;

  const auto state = leg_calc::wheelsFromBody(params, 2.0, 5.0);

  EXPECT_FALSE(state.saturated);
  EXPECT_NEAR(state.linear, 2.0, 1e-12);
  EXPECT_NEAR(
    state.left_speed,
    (2.0 - 0.5 * params.wheel_separation * 5.0) / params.wheel_radius, 1e-12);
}

// 参数非法时返回全零，不让 NaN 流到电机
TEST(DiffDrive, InvalidParamsGiveZeros)
{
  leg_calc::DiffDriveParams params = makeParams();
  params.wheel_radius = 0.0;

  EXPECT_FALSE(params.isValid());
  const auto wheels = leg_calc::wheelsFromBody(params, 0.5, 1.0);
  EXPECT_EQ(wheels.left_speed, 0.0);
  EXPECT_EQ(wheels.right_speed, 0.0);

  const auto body = leg_calc::bodyFromWheels(params, 1.0, 1.0);
  EXPECT_EQ(body.linear, 0.0);
  EXPECT_EQ(body.angular, 0.0);
}
