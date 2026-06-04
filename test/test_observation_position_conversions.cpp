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
#include "romea_localisation_utils/conversions/observation_position_conversions.hpp"

//-----------------------------------------------------------------------------
class TestObsPositionConversion : public ::testing::Test
{
public:
  TestObsPositionConversion()
  : stamp(1000), frame_id("foo"), romea_obs_position(), romea_obs_position_msg()
  {
  }

  void SetUp() override
  {
    romea_obs_position.Y(romea::core::localisation::ObservationPosition::POSITION_X) = 1;
    romea_obs_position.Y(romea::core::localisation::ObservationPosition::POSITION_Y) = 2;
    romea_obs_position.lever_arm.x() = 4;
    romea_obs_position.lever_arm.y() = 5;
    romea_obs_position.lever_arm.z() = 6;
    fillEigenCovariance(romea_obs_position.R());
    romea::ros2::to_ros_msg(stamp, frame_id, romea_obs_position, romea_obs_position_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::core::localisation::ObservationPosition romea_obs_position;
  romea_localisation_msgs::msg::ObservationPosition2DStamped romea_obs_position_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsPositionConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::ros2::extract_time(romea_obs_position_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_position_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(
    romea_obs_position_msg.observation_position.position.x,
    romea_obs_position.Y(romea::core::localisation::ObservationPosition::POSITION_X));
  EXPECT_DOUBLE_EQ(
    romea_obs_position_msg.observation_position.position.y,
    romea_obs_position.Y(romea::core::localisation::ObservationPosition::POSITION_Y));

  isSame(romea_obs_position_msg.observation_position.position.covariance, romea_obs_position.R());
}

//-----------------------------------------------------------------------------
TEST_F(TestObsPositionConversion, fromRosMsgtoObs)
{
  romea::core::localisation::ObservationPosition romea_obs_position_bis;
  romea::ros2::extract_obs(romea_obs_position_msg, romea_obs_position_bis);
  EXPECT_DOUBLE_EQ(
    romea_obs_position_bis.Y(romea::core::localisation::ObservationPosition::POSITION_X),
    romea_obs_position.Y(romea::core::localisation::ObservationPosition::POSITION_X));
  EXPECT_DOUBLE_EQ(
    romea_obs_position_bis.Y(romea::core::localisation::ObservationPosition::POSITION_Y),
    romea_obs_position.Y(romea::core::localisation::ObservationPosition::POSITION_Y));
  isSame(romea_obs_position_bis.R(), romea_obs_position.R());
}

//-----------------------------------------------------------------------------
class TestPositionConversion : public ::testing::Test
{
public:
  TestPositionConversion()
  : stamp(1000), frame_id("foo"), romea_position(), romea_obs_position_msg()
  {
  }

  void SetUp() override
  {
    romea_position.position.x() = 1;
    romea_position.position.y() = 2;
    fillEigenCovariance(romea_position.covariance);
    romea::ros2::to_ros_msg(stamp, frame_id, romea_position, romea_obs_position_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::core::Position2D romea_position;
  romea_localisation_msgs::msg::ObservationPosition2DStamped romea_obs_position_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestPositionConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::ros2::extract_time(romea_obs_position_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_position_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(
    romea_obs_position_msg.observation_position.position.x, romea_position.position.x());
  EXPECT_DOUBLE_EQ(
    romea_obs_position_msg.observation_position.position.y, romea_position.position.y());
  isSame(
    romea_obs_position_msg.observation_position.position.covariance, romea_position.covariance);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
