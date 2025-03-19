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

// romea
#include "romea_localisation_utils/conversions/observation_twist_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::Twist2D & twist,
  romea_localisation_msgs::msg::ObservationTwist2D & msg)
{
  ros2::to_ros_msg(twist, msg.twist);
  msg.lever_arm.x = 0;
  msg.lever_arm.y = 0;
  msg.lever_arm.z = 0;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::Twist2D & twist,
  romea_localisation_msgs::msg::ObservationTwist2DStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(twist, msg.observation_twist);
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::localisation::ObservationTwist & observation,
  romea_localisation_msgs::msg::ObservationTwist2D & msg)
{
  msg.twist.linear_speeds.x =
    observation.Y(core::localisation::ObservationTwist::LINEAR_SPEED_X_BODY);
  msg.twist.linear_speeds.y =
    observation.Y(core::localisation::ObservationTwist::LINEAR_SPEED_Y_BODY);
  msg.twist.angular_speed =
    observation.Y(core::localisation::ObservationTwist::ANGULAR_SPEED_Z_BODY);

  msg.lever_arm.x = observation.lever_arm.x();
  msg.lever_arm.y = observation.lever_arm.y();
  msg.lever_arm.z = observation.lever_arm.z();

  for (size_t n = 0; n < 9; ++n) {
    msg.twist.covariance[n] = observation.R()(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::localisation::ObservationTwist & observation,
  romea_localisation_msgs::msg::ObservationTwist2DStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(observation, msg.observation_twist);
}

//-----------------------------------------------------------------------------
void extract_obs(
  const romea_localisation_msgs::msg::ObservationTwist2DStamped & msg,
  core::localisation::ObservationTwist & observation)
{
  observation.Y(core::localisation::ObservationTwist::LINEAR_SPEED_X_BODY) =
    msg.observation_twist.twist.linear_speeds.x;
  observation.Y(core::localisation::ObservationTwist::LINEAR_SPEED_Y_BODY) =
    msg.observation_twist.twist.linear_speeds.y;
  observation.Y(core::localisation::ObservationTwist::ANGULAR_SPEED_Z_BODY) =
    msg.observation_twist.twist.angular_speed;
  observation.R() = Eigen::Matrix3d(msg.observation_twist.twist.covariance.data());
  observation.lever_arm.x() = msg.observation_twist.lever_arm.x;
  observation.lever_arm.y() = msg.observation_twist.lever_arm.y;
  observation.lever_arm.z() = msg.observation_twist.lever_arm.z;
}

}  // namespace ros2
}  // namespace romea
