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

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__FACTORY_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__FACTORY_HPP_

// std
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "parameters.hpp"
#include "romea_common_utils/log/topic_logger.hpp"
#include "romea_common_utils/params/algorithm_parameters.hpp"
#include "romea_core_common/fsm/FSMEventNotifier.hpp"
#include "romea_core_common/log/Logger.hpp"
#include "romea_core_localisation/updater_exteroceptive.hpp"
#include "romea_core_localisation/updater_proprioceptive.hpp"
#include "romea_core_localisation/updater_trigger_mode.hpp"

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

//-----------------------------------------------------------------------------
template<class Updater>
std::unique_ptr<Updater> make_kalman_exteroceptive_updater(
  std::shared_ptr<rclcpp::Node> & node,
  const std::string & updater_name,
  std::shared_ptr<core::Logger> logger,
  core::FSMEventCallback fsm_event_callback = nullptr)
{
  auto updater = std::make_unique<Updater>(
    updater_name,
    get_updater_minimal_rate(node, updater_name),
    core::localisation::to_trigger_mode(get_updater_trigger_mode(node, updater_name)),
    get_updater_mahalanobis_distance_rejection_threshold(node, updater_name));

  updater->register_logger(std::move(logger));
  updater->register_fsm_event_callback(std::move(fsm_event_callback));
  return updater;
}

//-----------------------------------------------------------------------------
template<class Updater>
std::unique_ptr<Updater> make_kalman_exteroceptive_updater(
  std::shared_ptr<rclcpp::Node> & node, const std::string & updater_name)
{
  return make_kalman_exteroceptive_updater<Updater>(
    node, updater_name, make_topic_logger(node, updater_name));
}

//-----------------------------------------------------------------------------
template<class Updater>
std::unique_ptr<Updater> make_particle_exteroceptive_updater(
  std::shared_ptr<rclcpp::Node> & node,
  const std::string & updater_name,
  std::shared_ptr<core::Logger> logger,
  core::FSMEventCallback fsm_event_callback = nullptr)
{
  auto updater = std::make_unique<Updater>(
    updater_name,
    get_updater_minimal_rate(node, updater_name),
    core::localisation::to_trigger_mode(get_updater_trigger_mode(node, updater_name)),
    get_filter_number_of_particles(node),
    get_updater_mahalanobis_distance_rejection_threshold(node, updater_name));

  updater->register_logger(std::move(logger));
  updater->register_fsm_event_callback(std::move(fsm_event_callback));
  return updater;
}

//-----------------------------------------------------------------------------
template<class Updater>
std::unique_ptr<Updater> make_particle_exteroceptive_updater(
  std::shared_ptr<rclcpp::Node> & node, const std::string & updater_name)
{
  return make_particle_exteroceptive_updater<Updater>(
    node, updater_name, make_topic_logger(node, updater_name));
}

//-----------------------------------------------------------------------------
template<class Updater, core::FilterType FilterType_>
std::unique_ptr<Updater> make_exteroceptive_updater(
  std::shared_ptr<rclcpp::Node> & node,
  const std::string & updater_name,
  std::shared_ptr<core::Logger> logger,
  core::FSMEventCallback fsm_event_callback = nullptr)
{
  if constexpr (FilterType_ == core::KALMAN) {
    return make_kalman_exteroceptive_updater<Updater>(
      node, updater_name, std::move(logger), std::move(fsm_event_callback));
  } else {
    return make_particle_exteroceptive_updater<Updater>(
      node, updater_name, std::move(logger), std::move(fsm_event_callback));
  }
}

//-----------------------------------------------------------------------------
template<class Updater, core::FilterType FilterType_>
std::unique_ptr<Updater> make_exteroceptive_updater(
  std::shared_ptr<rclcpp::Node> & node, const std::string & updater_name)
{
  return make_exteroceptive_updater<Updater, FilterType_>(
    node, updater_name, make_topic_logger(node, updater_name));
}

//-----------------------------------------------------------------------------
template<class Updater>
std::unique_ptr<Updater> make_proprioceptive_updater(
  std::shared_ptr<rclcpp::Node> & node, const std::string & updater_name)
{
  return std::make_unique<Updater>(updater_name, get_updater_minimal_rate(node, updater_name));
}

//-----------------------------------------------------------------------------
template<class Updater, core::FilterType FilterType_>
std::unique_ptr<Updater> make_updater(
  std::shared_ptr<rclcpp::Node> & node,
  const std::string & updater_name,
  std::shared_ptr<core::Logger> logger,
  core::FSMEventCallback fsm_event_callback = nullptr)
{
  if constexpr (std::is_base_of_v<core::localisation::UpdaterProprioceptive, Updater>) {
    return make_proprioceptive_updater<Updater>(node, updater_name);
  } else if constexpr (std::is_base_of_v<core::localisation::UpdaterExteroceptive, Updater>) {
    return make_exteroceptive_updater<Updater, FilterType_>(
      node, updater_name, std::move(logger), std::move(fsm_event_callback));
  } else {
    static_assert(
      std::is_base_of_v<core::localisation::UpdaterProprioceptive, Updater> ||
      std::is_base_of_v<core::localisation::UpdaterExteroceptive, Updater>,
      "Updater must inherit from UpdaterProprioceptive or UpdaterExteroceptive");
  }
}

//-----------------------------------------------------------------------------
template<class Predictor>
std::unique_ptr<Predictor> make_kalman_predictor(std::shared_ptr<rclcpp::Node> node)
{
  return std::make_unique<Predictor>(get_dead_reckoning_limits(node));
}

//-----------------------------------------------------------------------------
template<class Predictor>
std::unique_ptr<Predictor> make_particle_predictor(std::shared_ptr<rclcpp::Node> node)
{
  return std::make_unique<Predictor>(
    get_filter_number_of_particles(node), get_dead_reckoning_limits(node));
}

//-----------------------------------------------------------------------------
template<class Predictor, core::FilterType FilterType_>
std::unique_ptr<Predictor> make_predictor(std::shared_ptr<rclcpp::Node> node)
{
  if constexpr (FilterType_ == core::FilterType::KALMAN) {
    return make_kalman_predictor<Predictor>(node);
  } else {
    return make_particle_predictor<Predictor>(node);
  }
}

//-----------------------------------------------------------------------------
template<class Filter, core::FilterType FilterType_>
std::unique_ptr<Filter> make_filter(
  std::shared_ptr<rclcpp::Node> node,
  std::unique_ptr<typename Filter::Predictor> predictor)
{
  if constexpr (FilterType_ == core::FilterType::KALMAN) {
    return std::make_unique<Filter>(get_filter_state_pool_size(node), std::move(predictor));
  } else {
    return std::make_unique<Filter>(
      get_filter_state_pool_size(node), get_filter_number_of_particles(node), std::move(predictor));
  }
}

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__FACTORY_HPP_
