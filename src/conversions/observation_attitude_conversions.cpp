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
#include "romea_localisation_utils/conversions/observation_attitude_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::localisation::ObservationAttitude & observation,
  romea_localisation_msgs::msg::ObservationAttitude & msg)
{
  msg.roll_angle = observation.Y(core::localisation::ObservationAttitude::ROLL);
  msg.pitch_angle = observation.Y(core::localisation::ObservationAttitude::PITCH);

  for (size_t n = 0; n < 4; ++n) {
    msg.covariance[n] = observation.R()(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::localisation::ObservationAttitude & observation,
  romea_localisation_msgs::msg::ObservationAttitudeStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(observation, msg.observation_attitude);
}

//-----------------------------------------------------------------------------
void extract_obs(
  const romea_localisation_msgs::msg::ObservationAttitudeStamped & msg,
  core::localisation::ObservationAttitude & observation)
{
  observation.Y(core::localisation::ObservationAttitude::ROLL) =
    msg.observation_attitude.roll_angle;
  observation.Y(core::localisation::ObservationAttitude::PITCH) =
    msg.observation_attitude.pitch_angle;
  observation.R() = Eigen::Matrix2d(msg.observation_attitude.covariance.data());
}

}  // namespace ros2
}  // namespace romea
