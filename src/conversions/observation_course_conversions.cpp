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
#include "romea_localisation_utils/conversions/observation_course_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::ObservationCourse & observation,
  romea_localisation_msgs::msg::ObservationCourse & msg)
{
  msg.angle = observation.Y();
  msg.std = std::sqrt(observation.R());
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::ObservationCourse & observation,
  romea_localisation_msgs::msg::ObservationCourseStamped & msg)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  to_ros_msg(observation, msg.observation_course);
}

//-----------------------------------------------------------------------------
void extract_obs(
  const romea_localisation_msgs::msg::ObservationCourseStamped & msg,
  core::ObservationCourse & observation)
{
  observation.Y() = msg.observation_course.angle;
  observation.R() = msg.observation_course.std * msg.observation_course.std;
}

}  // namespace ros2
}  // namespace romea
