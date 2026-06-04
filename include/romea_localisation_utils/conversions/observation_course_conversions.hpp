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

#ifndef ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_COURSE_CONVERSIONS_HPP_
#define ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_COURSE_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include "romea_common_utils/conversions/time_conversions.hpp"
#include "romea_core_localisation/observation_course.hpp"
#include "romea_localisation_msgs/msg/observation_course_stamped.hpp"

namespace romea
{
namespace ros2
{

void to_ros_msg(
  const core::localisation::ObservationCourse & observation,
  romea_localisation_msgs::msg::ObservationCourse & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::localisation::ObservationCourse & observation,
  romea_localisation_msgs::msg::ObservationCourseStamped & msg);

void extract_obs(
  const romea_localisation_msgs::msg::ObservationCourseStamped & msg,
  core::localisation::ObservationCourse & observation);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_COURSE_CONVERSIONS_HPP_
