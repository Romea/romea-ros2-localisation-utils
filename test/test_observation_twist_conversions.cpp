// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <string>

// gtest
#include "gtest/gtest.h"

// romea
#include "../test/test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_twist_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsTwistConversion : public ::testing::Test
{
public:
  TestObsTwistConversion()
  : stamp(1000),
    frame_id("foo"),
    romea_obs_twist(),
    romea_obs_twist_msg()
  {
  }

  void SetUp()override
  {
    romea_obs_twist.Y(romea::core::localisation::ObservationTwist::LINEAR_SPEED_X_BODY) = 1;
    romea_obs_twist.Y(romea::core::localisation::ObservationTwist::LINEAR_SPEED_Y_BODY) = 2;
    romea_obs_twist.Y(romea::core::localisation::ObservationTwist::ANGULAR_SPEED_Z_BODY) = 3;
    romea_obs_twist.lever_arm.x() = 4;
    romea_obs_twist.lever_arm.y() = 5;
    romea_obs_twist.lever_arm.z() = 6;
    fillEigenCovariance(romea_obs_twist.R());
    romea::ros2::to_ros_msg(stamp, frame_id, romea_obs_twist, romea_obs_twist_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::core::localisation::ObservationTwist romea_obs_twist;
  romea_localisation_msgs::msg::ObservationTwist2DStamped romea_obs_twist_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsTwistConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::ros2::extract_time(romea_obs_twist_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_twist_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_msg.observation_twist.twist.linear_speeds.x,
    romea_obs_twist.Y(romea::core::localisation::ObservationTwist::LINEAR_SPEED_X_BODY));
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_msg.observation_twist.twist.linear_speeds.y,
    romea_obs_twist.Y(romea::core::localisation::ObservationTwist::LINEAR_SPEED_Y_BODY));
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_msg.observation_twist.twist.angular_speed,
    romea_obs_twist.Y(romea::core::localisation::ObservationTwist::ANGULAR_SPEED_Z_BODY));
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_msg.observation_twist.lever_arm.x,
    romea_obs_twist.lever_arm.x());
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_msg.observation_twist.lever_arm.y,
    romea_obs_twist.lever_arm.y());
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_msg.observation_twist.lever_arm.z,
    romea_obs_twist.lever_arm.z());
  isSame(romea_obs_twist_msg.observation_twist.twist.covariance, romea_obs_twist.R());
}

//-----------------------------------------------------------------------------
TEST_F(TestObsTwistConversion, fromRosMsgtoObs)
{
  romea::core::localisation::ObservationTwist romea_obs_twist_bis;
  romea::ros2::extract_obs(romea_obs_twist_msg, romea_obs_twist_bis);
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_bis.Y(romea::core::localisation::ObservationTwist::LINEAR_SPEED_X_BODY),
    romea_obs_twist.Y(romea::core::localisation::ObservationTwist::LINEAR_SPEED_X_BODY));
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_bis.Y(romea::core::localisation::ObservationTwist::LINEAR_SPEED_Y_BODY),
    romea_obs_twist.Y(romea::core::localisation::ObservationTwist::LINEAR_SPEED_Y_BODY));
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_bis.Y(romea::core::localisation::ObservationTwist::ANGULAR_SPEED_Z_BODY),
    romea_obs_twist.Y(romea::core::localisation::ObservationTwist::ANGULAR_SPEED_Z_BODY));
  EXPECT_DOUBLE_EQ(romea_obs_twist_bis.lever_arm.x(), romea_obs_twist.lever_arm.x());
  EXPECT_DOUBLE_EQ(romea_obs_twist_bis.lever_arm.y(), romea_obs_twist.lever_arm.y());
  EXPECT_DOUBLE_EQ(romea_obs_twist_bis.lever_arm.z(), romea_obs_twist.lever_arm.z());
  isSame(romea_obs_twist_bis.R(), romea_obs_twist.R());
}


//-----------------------------------------------------------------------------
class TestPoseConversion : public ::testing::Test
{
public:
  TestPoseConversion()
  : stamp(1000),
    frame_id("foo"),
    romea_twist(),
    romea_obs_twist_msg()
  {
  }

  void SetUp()override
  {
    romea_twist.linearSpeeds.x() = 1;
    romea_twist.linearSpeeds.y() = 2;
    romea_twist.angularSpeed = 3;
    fillEigenCovariance(romea_twist.covariance);
    romea::ros2::to_ros_msg(stamp, frame_id, romea_twist, romea_obs_twist_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::core::Twist2D romea_twist;
  romea_localisation_msgs::msg::ObservationTwist2DStamped romea_obs_twist_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestPoseConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::ros2::extract_time(romea_obs_twist_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_twist_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_msg.observation_twist.twist.linear_speeds.x,
    romea_twist.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_msg.observation_twist.twist.linear_speeds.y,
    romea_twist.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(
    romea_obs_twist_msg.observation_twist.twist.angular_speed,
    romea_twist.angularSpeed);
  EXPECT_DOUBLE_EQ(romea_obs_twist_msg.observation_twist.lever_arm.x, 0);
  EXPECT_DOUBLE_EQ(romea_obs_twist_msg.observation_twist.lever_arm.y, 0);
  EXPECT_DOUBLE_EQ(romea_obs_twist_msg.observation_twist.lever_arm.z, 0);
  isSame(romea_obs_twist_msg.observation_twist.twist.covariance, romea_twist.covariance);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
