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

#ifndef ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_LINEAR_SPEED_CONVERSIONS_HPP_
#define ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_LINEAR_SPEED_CONVERSIONS_HPP_

#include "romea_localisation_msgs/msg/observation_twist2_d_stamped.hpp"
#include "romea_core_localisation/observation_linear_speed.hpp"

namespace romea
{
namespace ros2
{

void extract_obs(
  const romea_localisation_msgs::msg::ObservationTwist2DStamped & msg,
  core::localisation::ObservationLinearSpeed & observation);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_LINEAR_SPEED_CONVERSIONS_HPP_
