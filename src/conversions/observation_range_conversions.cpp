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
#include "romea_localisation_utils/conversions/observation_range_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::localisation::ObservationRange & observation,
  romea_localisation_msgs::msg::ObservationRange & msg)
{
  msg.range = observation.Y();
  msg.range_std = std::sqrt(observation.R());
  msg.initiator_antenna_position.x = observation.initiator_position.x();
  msg.initiator_antenna_position.y = observation.initiator_position.y();
  msg.initiator_antenna_position.z = observation.initiator_position.z();
  msg.responder_antenna_position.x = observation.responder_position.x();
  msg.responder_antenna_position.y = observation.responder_position.y();
  msg.responder_antenna_position.z = observation.responder_position.z();
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::localisation::ObservationRange & observation,
  romea_localisation_msgs::msg::ObservationRangeStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(observation, msg.observation_range);
}

//-----------------------------------------------------------------------------
void extract_obs(
  const romea_localisation_msgs::msg::ObservationRangeStamped & msg,
  core::localisation::ObservationRange & observation)
{
  observation.Y() = msg.observation_range.range;
  observation.R() = msg.observation_range.range_std * msg.observation_range.range_std;
  observation.initiator_position.x() = msg.observation_range.initiator_antenna_position.x;
  observation.initiator_position.y() = msg.observation_range.initiator_antenna_position.y;
  observation.initiator_position.z() = msg.observation_range.initiator_antenna_position.z;
  observation.responder_position.x() = msg.observation_range.responder_antenna_position.x;
  observation.responder_position.y() = msg.observation_range.responder_antenna_position.y;
  observation.responder_position.z() = msg.observation_range.responder_antenna_position.z;
}

}  // namespace ros2
}  // namespace romea
