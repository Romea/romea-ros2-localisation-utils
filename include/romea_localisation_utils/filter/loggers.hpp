// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__LOGGERS_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__LOGGERS_HPP_

// std
#include <memory>
#include <string>

#include "romea_common_utils/log/topic_logger.hpp"
#include "romea_common_utils/params/algorithm_parameters.hpp"
#include "romea_core_common/log/Logger.hpp"

namespace romea
{
namespace ros2
{
namespace localisation
{

//-----------------------------------------------------------------------------
template<class NodeType>
std::shared_ptr<core::Logger> make_topic_logger(
  std::shared_ptr<NodeType> node, const std::string & logger_name)
{
  if (!get_debug(node)) {
    return nullptr;
  }

  using Logger = TopicLogger<NodeType>;
  return std::make_shared<Logger>(
    node, "debug/" + logger_name, rclcpp::SystemDefaultsQoS(), typename Logger::Options(), true);
}

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__LOGGERS_HPP_
