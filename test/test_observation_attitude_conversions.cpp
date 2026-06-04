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
#include "romea_localisation_utils/conversions/observation_attitude_conversions.hpp"
#include "test_utils.hpp"

//-----------------------------------------------------------------------------
class TestObsAttitudeConversion : public ::testing::Test
{
public:
  TestObsAttitudeConversion()
  : stamp(1000), frame_id("foo"), romea_obs_attitude(), romea_obs_attitude_msg()
  {
  }

  void SetUp() override
  {
    romea_obs_attitude.Y(romea::core::localisation::ObservationAttitude::ROLL) = 1;
    romea_obs_attitude.Y(romea::core::localisation::ObservationAttitude::PITCH) = 2;
    fillEigenCovariance(romea_obs_attitude.R());
    romea::ros2::to_ros_msg(stamp, frame_id, romea_obs_attitude, romea_obs_attitude_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::core::localisation::ObservationAttitude romea_obs_attitude;
  romea_localisation_msgs::msg::ObservationAttitudeStamped romea_obs_attitude_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsAttitudeConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::ros2::extract_time(romea_obs_attitude_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_attitude_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(
    romea_obs_attitude_msg.observation_attitude.roll_angle,
    romea_obs_attitude.Y(romea::core::localisation::ObservationAttitude::ROLL));
  EXPECT_DOUBLE_EQ(
    romea_obs_attitude_msg.observation_attitude.pitch_angle,
    romea_obs_attitude.Y(romea::core::localisation::ObservationAttitude::PITCH));

  isSame(romea_obs_attitude_msg.observation_attitude.covariance, romea_obs_attitude.R());
}

//-----------------------------------------------------------------------------
TEST_F(TestObsAttitudeConversion, fromRosMsgtoObs)
{
  romea::core::localisation::ObservationAttitude romea_obs_attitude_bis;
  romea::ros2::extract_obs(romea_obs_attitude_msg, romea_obs_attitude_bis);

  EXPECT_DOUBLE_EQ(
    romea_obs_attitude_bis.Y(romea::core::localisation::ObservationAttitude::ROLL),
    romea_obs_attitude.Y(romea::core::localisation::ObservationAttitude::ROLL));
  EXPECT_DOUBLE_EQ(
    romea_obs_attitude_bis.Y(romea::core::localisation::ObservationAttitude::PITCH),
    romea_obs_attitude.Y(romea::core::localisation::ObservationAttitude::PITCH));

  isSame(romea_obs_attitude_bis.R(), romea_obs_attitude.R());
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
