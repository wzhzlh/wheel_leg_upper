#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "leg_calc/leg_control_io.hpp"

namespace
{

const std::vector<std::string> kLegNames{"lf", "lb", "rf", "rb"};
const std::vector<std::string> kWheelNames{"left", "right"};

sensor_msgs::msg::JointState validJoints()
{
  sensor_msgs::msg::JointState msg;
  msg.name = {"lf", "lb", "rf", "rb", "left", "right"};
  msg.position = {0.1, 0.2, 0.3, 0.4, 1.0, 1.0};
  msg.velocity = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0};
  return msg;
}

}  // namespace

TEST(LegControlIO, RejectsInvalidImuOrientation)
{
  leg_calc::SnapshotIO io;
  sensor_msgs::msg::Imu imu;
  imu.orientation.w = 1.0;
  io.onImu(imu, 1, 0.0, 1.0);
  EXPECT_TRUE(io.build(1.01, 0.03).imu_ok);

  imu.orientation.w = 0.0;
  io.onImu(imu, 1, 0.0, 1.02);
  EXPECT_FALSE(io.build(1.03, 0.03).imu_ok);
}

TEST(LegControlIO, RejectsNonFiniteJointAndWheelFeedback)
{
  leg_calc::SnapshotIO io;
  auto msg = validJoints();
  io.onJointStates(msg, kLegNames, kWheelNames, 1.0);
  auto snapshot = io.build(1.01, 0.03);
  EXPECT_TRUE(snapshot.joint_ok);
  EXPECT_TRUE(snapshot.wheel_ok);

  msg.position[4] = std::numeric_limits<double>::quiet_NaN();
  io.onJointStates(msg, kLegNames, kWheelNames, 1.02);
  snapshot = io.build(1.03, 0.03);
  EXPECT_FALSE(snapshot.wheel_ok);
  EXPECT_TRUE(std::isfinite(snapshot.x));

  msg = validJoints();
  msg.velocity[0] = std::numeric_limits<double>::quiet_NaN();
  io.onJointStates(msg, kLegNames, kWheelNames, 1.04);
  EXPECT_FALSE(io.build(1.05, 0.03).joint_ok);
}

TEST(LegControlIO, ForeignJointStatesDoNotInvalidateFeedback)
{
  // 同一个 /joint_states 上还有别的机器人时，不能因为"消息里没有本机关节"就判本机反馈失效
  leg_calc::SnapshotIO io;
  io.onJointStates(validJoints(), kLegNames, kWheelNames, 1.0);
  EXPECT_TRUE(io.build(1.01, 0.03).joint_ok);

  sensor_msgs::msg::JointState foreign;
  foreign.name = {"base_to_chassis", "front_left_steer_joint", "front_left_drive_joint"};
  foreign.position = {0.0, 0.1, 0.2};
  foreign.velocity = {0.0, 0.0, 0.0};
  io.onJointStates(foreign, kLegNames, kWheelNames, 1.02);

  const auto snapshot = io.build(1.03, 0.03);
  EXPECT_TRUE(snapshot.joint_ok);
  EXPECT_TRUE(snapshot.wheel_ok);
  EXPECT_NEAR(snapshot.joint_age, 0.03, 1e-12);   // 时间戳不能被别人的消息刷新
  EXPECT_NEAR(snapshot.wheel_age, 0.03, 1e-12);
}

TEST(LegControlIO, InvalidCommandDoesNotRefreshAge)
{
  leg_calc::SnapshotIO io;
  std_msgs::msg::Float64MultiArray command;
  command.data = {0.0, 0.0, 0.145, 0.0, 1.0, 0.0};
  io.onCommand(command, 1.0);

  command.data[3] = std::numeric_limits<double>::quiet_NaN();
  io.onCommand(command, 1.02);
  const auto snapshot = io.build(1.05, 0.03);
  EXPECT_TRUE(snapshot.command_seen);
  EXPECT_NEAR(snapshot.command_age, 0.05, 1e-12);
}
