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
#include "romea_localisation_utils/conversions/observation_linear_speed_conversions.hpp"

//-----------------------------------------------------------------------------
class TestObsLinearSpeedConversion : public ::testing::Test
{
public:
  TestObsLinearSpeedConversion()
  : romea_obs_linear_speed(),
    ros_obs_linear_speed_msg()
  {
  }

  void SetUp()override
  {
    ros_obs_linear_speed_msg.observation_twist.twist.linear_speeds.x = 1;
    ros_obs_linear_speed_msg.observation_twist.twist.linear_speeds.y = 2;
    ros_obs_linear_speed_msg.observation_twist.twist.angular_speed = 3;
    ros_obs_linear_speed_msg.observation_twist.lever_arm.x = 4;
    ros_obs_linear_speed_msg.observation_twist.lever_arm.x = 5;
    ros_obs_linear_speed_msg.observation_twist.lever_arm.x = 6;
    fillMsgCovariance(ros_obs_linear_speed_msg.observation_twist.twist.covariance);
  }

  romea::core::localisation::ObservationLinearSpeed romea_obs_linear_speed;
  romea_localisation_msgs::msg::ObservationTwist2DStamped ros_obs_linear_speed_msg;
};


//-----------------------------------------------------------------------------
TEST_F(TestObsLinearSpeedConversion, fromRosMsgtoObs)
{
  romea::core::localisation::ObservationLinearSpeed romea_obs_linear_speed;
  romea::ros2::extract_obs(ros_obs_linear_speed_msg, romea_obs_linear_speed);
  EXPECT_DOUBLE_EQ(
    romea_obs_linear_speed.Y(),
    ros_obs_linear_speed_msg.observation_twist.twist.linear_speeds.x);
  EXPECT_DOUBLE_EQ(
    romea_obs_linear_speed.R(),
    ros_obs_linear_speed_msg.observation_twist.twist.covariance[0]);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
