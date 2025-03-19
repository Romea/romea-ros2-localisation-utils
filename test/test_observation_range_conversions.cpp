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
#include "test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_range_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsRangeConversion : public ::testing::Test
{
public:
  TestObsRangeConversion()
  : stamp(1000),
    frame_id("foo"),
    romea_obs_range(),
    ros_obs_range_msg()
  {
  }

  void SetUp()override
  {
    romea_obs_range.Y() = 1;
    romea_obs_range.R() = 4;
    romea_obs_range.initiator_position.x() = 1;
    romea_obs_range.initiator_position.y() = 2;
    romea_obs_range.initiator_position.z() = 3;
    romea_obs_range.responder_position.x() = 4;
    romea_obs_range.responder_position.y() = 5;
    romea_obs_range.responder_position.z() = 6;
    romea::ros2::to_ros_msg(stamp, frame_id, romea_obs_range, ros_obs_range_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::core::localisation::ObservationRange romea_obs_range;
  romea_localisation_msgs::msg::ObservationRangeStamped ros_obs_range_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsRangeConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::ros2::extract_time(ros_obs_range_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(ros_obs_range_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_obs_range_msg.observation_range.range, romea_obs_range.Y());
  EXPECT_DOUBLE_EQ(ros_obs_range_msg.observation_range.range_std, std::sqrt(romea_obs_range.R()));
  EXPECT_DOUBLE_EQ(
    ros_obs_range_msg.observation_range.initiator_antenna_position.x,
    romea_obs_range.initiator_position.x());
  EXPECT_DOUBLE_EQ(
    ros_obs_range_msg.observation_range.initiator_antenna_position.y,
    romea_obs_range.initiator_position.y());
  EXPECT_DOUBLE_EQ(
    ros_obs_range_msg.observation_range.initiator_antenna_position.z,
    romea_obs_range.initiator_position.z());
  EXPECT_DOUBLE_EQ(
    ros_obs_range_msg.observation_range.responder_antenna_position.x,
    romea_obs_range.responder_position.x());
  EXPECT_DOUBLE_EQ(
    ros_obs_range_msg.observation_range.responder_antenna_position.y,
    romea_obs_range.responder_position.y());
  EXPECT_DOUBLE_EQ(
    ros_obs_range_msg.observation_range.responder_antenna_position.z,
    romea_obs_range.responder_position.z());
}

//-----------------------------------------------------------------------------
TEST_F(TestObsRangeConversion, fromRosMsgtoObs)
{
  romea::core::localisation::ObservationRange romea_obs_range_bis;
  romea::ros2::extract_obs(ros_obs_range_msg, romea_obs_range_bis);
  EXPECT_DOUBLE_EQ(romea_obs_range_bis.Y(), romea_obs_range.Y());
  EXPECT_DOUBLE_EQ(romea_obs_range_bis.R(), romea_obs_range.R());
  EXPECT_DOUBLE_EQ(
    romea_obs_range_bis.initiator_position.x(),
    romea_obs_range.initiator_position.x());
  EXPECT_DOUBLE_EQ(
    romea_obs_range_bis.initiator_position.y(),
    romea_obs_range.initiator_position.y());
  EXPECT_DOUBLE_EQ(
    romea_obs_range_bis.initiator_position.z(),
    romea_obs_range.initiator_position.z());
  EXPECT_DOUBLE_EQ(
    romea_obs_range_bis.responder_position.x(),
    romea_obs_range.responder_position.x());
  EXPECT_DOUBLE_EQ(
    romea_obs_range_bis.responder_position.y(),
    romea_obs_range.responder_position.y());
  EXPECT_DOUBLE_EQ(
    romea_obs_range_bis.responder_position.z(),
    romea_obs_range.responder_position.z());
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
