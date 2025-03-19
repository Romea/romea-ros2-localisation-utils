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
#include "romea_localisation_utils/conversions/observation_pose_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::Pose2D & pose,
  romea_localisation_msgs::msg::ObservationPose2D & msg)
{
  ros2::to_ros_msg(pose, msg.pose);
  msg.lever_arm.x = 0;
  msg.lever_arm.y = 0;
  msg.lever_arm.z = 0;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::Pose2D & pose,
  romea_localisation_msgs::msg::ObservationPose2DStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(pose, msg.observation_pose);
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::localisation::ObservationPose & observation,
  romea_localisation_msgs::msg::ObservationPose2D & msg)
{
  msg.pose.position.x = observation.Y(core::localisation::ObservationPose::POSITION_X);
  msg.pose.position.y = observation.Y(core::localisation::ObservationPose::POSITION_Y);
  msg.pose.yaw = observation.Y(core::localisation::ObservationPose::ORIENTATION_Z);
  msg.lever_arm.x = observation.lever_arm.x();
  msg.lever_arm.y = observation.lever_arm.y();
  msg.lever_arm.z = observation.lever_arm.z();

  for (size_t n = 0; n < 9; ++n) {
    msg.pose.covariance[n] = observation.R()(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::localisation::ObservationPose & observation,
  romea_localisation_msgs::msg::ObservationPose2DStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(observation, msg.observation_pose);
}

//-----------------------------------------------------------------------------
void extract_obs(
  const romea_localisation_msgs::msg::ObservationPose2DStamped & msg,
  core::localisation::ObservationPose & observation)
{
  observation.Y(core::localisation::ObservationPose::POSITION_X) =
    msg.observation_pose.pose.position.x;
  observation.Y(core::localisation::ObservationPose::POSITION_Y) =
    msg.observation_pose.pose.position.y;
  observation.Y(core::localisation::ObservationPose::ORIENTATION_Z) =
    msg.observation_pose.pose.yaw;

  observation.R() = Eigen::Matrix3d(msg.observation_pose.pose.covariance.data());
  observation.lever_arm.x() = msg.observation_pose.lever_arm.x;
  observation.lever_arm.y() = msg.observation_pose.lever_arm.y;
  observation.lever_arm.z() = msg.observation_pose.lever_arm.z;
}
}  // namespace ros2
}  // namespace romea
