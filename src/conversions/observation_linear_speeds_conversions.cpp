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

#include "romea_localisation_utils/conversions/observation_linear_speeds_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void extract_obs(
  const romea_localisation_msgs::msg::ObservationTwist2DStamped & msg,
  core::localisation::ObservationLinearSpeeds & observation)
{
  observation.Y(core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY) =
    msg.observation_twist.twist.linear_speeds.x;
  observation.Y(core::localisation::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY) =
    msg.observation_twist.twist.linear_speeds.y;
  observation.R(0, 0) = msg.observation_twist.twist.covariance[0],
  observation.R(0, 1) = msg.observation_twist.twist.covariance[1];
  observation.R(1, 0) = msg.observation_twist.twist.covariance[3],
  observation.R(1, 1) = msg.observation_twist.twist.covariance[4];
}

}  // namespace ros2
}  // namespace romea
