// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_RANGE_CONVERSIONS_HPP_
#define ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_RANGE_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include "romea_core_localisation/ObservationRange.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"
#include "romea_localisation_msgs/msg/observation_range_stamped.hpp"

namespace romea
{

void to_ros_msg(
  const ObservationRange & observation,
  romea_localisation_msgs::msg::ObservationRange & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const ObservationRange & observation,
  romea_localisation_msgs::msg::ObservationRangeStamped & msg);

void extract_obs(
  const romea_localisation_msgs::msg::ObservationRangeStamped & msg,
  ObservationRange & observation);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_RANGE_CONVERSIONS_HPP_
