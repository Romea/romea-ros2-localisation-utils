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

// gtest
#include "gtest/gtest.h"

// romea
#include "../test/test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_linear_speeds_conversions.hpp"

//-----------------------------------------------------------------------------
class TestObsLinearSpeedsConversion : public ::testing::Test
{
public:
  TestObsLinearSpeedsConversion() : romea_obs_linear_speeds(), ros_obs_linear_speeds_msg() {}

  void SetUp() override
  {
    ros_obs_linear_speeds_msg.observation_twist.twist.linear_speeds.x = 1;
    ros_obs_linear_speeds_msg.observation_twist.twist.linear_speeds.y = 2;
    ros_obs_linear_speeds_msg.observation_twist.twist.angular_speed = 3;
    ros_obs_linear_speeds_msg.observation_twist.lever_arm.x = 4;
    ros_obs_linear_speeds_msg.observation_twist.lever_arm.x = 5;
    ros_obs_linear_speeds_msg.observation_twist.lever_arm.x = 6;
    fillMsgCovariance(ros_obs_linear_speeds_msg.observation_twist.twist.covariance);
  }

  romea::core::localisation::ObservationLinearSpeeds romea_obs_linear_speeds;
  romea_localisation_msgs::msg::ObservationTwist2DStamped ros_obs_linear_speeds_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsLinearSpeedsConversion, fromRosMsgtoObs)
{
  romea::core::localisation::ObservationLinearSpeeds romea_obs_linear_speeds;
  romea::ros2::extract_obs(ros_obs_linear_speeds_msg, romea_obs_linear_speeds);
  EXPECT_DOUBLE_EQ(
    romea_obs_linear_speeds.Y(
      romea::core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY),
    ros_obs_linear_speeds_msg.observation_twist.twist.linear_speeds.x);
  EXPECT_DOUBLE_EQ(
    romea_obs_linear_speeds.Y(
      romea::core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY),
    ros_obs_linear_speeds_msg.observation_twist.twist.linear_speeds.y);
  EXPECT_DOUBLE_EQ(
    romea_obs_linear_speeds.R(
      romea::core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY,
      romea::core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY),
    ros_obs_linear_speeds_msg.observation_twist.twist.covariance[0]);
  EXPECT_DOUBLE_EQ(
    romea_obs_linear_speeds.R(
      romea::core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY,
      romea::core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY),
    ros_obs_linear_speeds_msg.observation_twist.twist.covariance[1]);
  EXPECT_DOUBLE_EQ(
    romea_obs_linear_speeds.R(
      romea::core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY,
      romea::core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY),
    ros_obs_linear_speeds_msg.observation_twist.twist.covariance[3]);
  EXPECT_DOUBLE_EQ(
    romea_obs_linear_speeds.R(
      romea::core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY,
      romea::core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY),
    ros_obs_linear_speeds_msg.observation_twist.twist.covariance[4]);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
