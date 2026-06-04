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
#include "romea_localisation_utils/conversions/observation_pose_conversions.hpp"

//-----------------------------------------------------------------------------
class TestObsPoseConversion : public ::testing::Test
{
public:
  TestObsPoseConversion() : stamp(1000), frame_id("foo"), romea_obs_pose(), romea_obs_pose_msg() {}

  void SetUp() override
  {
    romea_obs_pose.Y(romea::core::localisation::ObservationPose::POSITION_X) = 1;
    romea_obs_pose.Y(romea::core::localisation::ObservationPose::POSITION_Y) = 2;
    romea_obs_pose.Y(romea::core::localisation::ObservationPose::ORIENTATION_Z) = 3;
    romea_obs_pose.lever_arm.x() = 4;
    romea_obs_pose.lever_arm.y() = 5;
    romea_obs_pose.lever_arm.z() = 6;
    fillEigenCovariance(romea_obs_pose.R());
    romea::ros2::to_ros_msg(stamp, frame_id, romea_obs_pose, romea_obs_pose_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::core::localisation::ObservationPose romea_obs_pose;
  romea_localisation_msgs::msg::ObservationPose2DStamped romea_obs_pose_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsPoseConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::ros2::extract_time(romea_obs_pose_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_pose_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(
    romea_obs_pose_msg.observation_pose.pose.position.x,
    romea_obs_pose.Y(romea::core::localisation::ObservationPose::POSITION_X));
  EXPECT_DOUBLE_EQ(
    romea_obs_pose_msg.observation_pose.pose.position.y,
    romea_obs_pose.Y(romea::core::localisation::ObservationPose::POSITION_Y));
  EXPECT_DOUBLE_EQ(
    romea_obs_pose_msg.observation_pose.pose.yaw,
    romea_obs_pose.Y(romea::core::localisation::ObservationPose::ORIENTATION_Z));
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.lever_arm.x, romea_obs_pose.lever_arm.x());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.lever_arm.y, romea_obs_pose.lever_arm.y());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.lever_arm.z, romea_obs_pose.lever_arm.z());

  isSame(romea_obs_pose_msg.observation_pose.pose.covariance, romea_obs_pose.R());
}

//-----------------------------------------------------------------------------
TEST_F(TestObsPoseConversion, fromRosMsgtoObs)
{
  romea::core::localisation::ObservationPose romea_obs_pose_bis;
  romea::ros2::extract_obs(romea_obs_pose_msg, romea_obs_pose_bis);
  EXPECT_DOUBLE_EQ(
    romea_obs_pose_bis.Y(romea::core::localisation::ObservationPose::POSITION_X),
    romea_obs_pose.Y(romea::core::localisation::ObservationPose::POSITION_X));
  EXPECT_DOUBLE_EQ(
    romea_obs_pose_bis.Y(romea::core::localisation::ObservationPose::POSITION_Y),
    romea_obs_pose.Y(romea::core::localisation::ObservationPose::POSITION_Y));
  EXPECT_DOUBLE_EQ(
    romea_obs_pose_bis.Y(romea::core::localisation::ObservationPose::ORIENTATION_Z),
    romea_obs_pose.Y(romea::core::localisation::ObservationPose::ORIENTATION_Z));
  EXPECT_DOUBLE_EQ(romea_obs_pose_bis.lever_arm.x(), romea_obs_pose.lever_arm.x());
  EXPECT_DOUBLE_EQ(romea_obs_pose_bis.lever_arm.y(), romea_obs_pose.lever_arm.y());
  EXPECT_DOUBLE_EQ(romea_obs_pose_bis.lever_arm.z(), romea_obs_pose.lever_arm.z());

  isSame(romea_obs_pose_bis.R(), romea_obs_pose.R());
}

//-----------------------------------------------------------------------------
class TestPoseConversion : public ::testing::Test
{
public:
  TestPoseConversion() : stamp(1000), frame_id("foo"), romea_pose(), romea_obs_pose_msg() {}

  void SetUp() override
  {
    romea_pose.position.x() = 1;
    romea_pose.position.y() = 2;
    romea_pose.yaw = 3;
    fillEigenCovariance(romea_pose.covariance);
    romea::ros2::to_ros_msg(stamp, frame_id, romea_pose, romea_obs_pose_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::core::Pose2D romea_pose;
  romea_localisation_msgs::msg::ObservationPose2DStamped romea_obs_pose_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestPoseConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::ros2::extract_time(romea_obs_pose_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_pose_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.pose.position.x, romea_pose.position.x());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.pose.position.y, romea_pose.position.y());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.pose.yaw, romea_pose.yaw);
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.lever_arm.x, 0);
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.lever_arm.y, 0);
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.lever_arm.z, 0);
  isSame(romea_obs_pose_msg.observation_pose.pose.covariance, romea_pose.covariance);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
