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

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__UPDATER_INTERFACE_BASE_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__UPDATER_INTERFACE_BASE_HPP_

// ros
#include <rclcpp/rclcpp.hpp>

// romea
#include <romea_common_utils/conversions/time_conversions.hpp>
#include <romea_core_common/diagnostic/CheckupRate.hpp>

namespace romea
{
namespace ros2
{
namespace localisation
{

class UpdaterInterfaceBase
{
public:
  UpdaterInterfaceBase() {}

  virtual bool heartbeat_callback(const core::Duration & duration) = 0;

  virtual core::DiagnosticReport get_report() = 0;
};

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__UPDATER_INTERFACE_BASE_HPP_
