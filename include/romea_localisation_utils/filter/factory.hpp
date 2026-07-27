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
#include <utility>

#include "parameters.hpp"
#include "romea_common_utils/log/topic_logger.hpp"
#include "romea_common_utils/params/algorithm_parameters.hpp"
#include "romea_core_common/fsm/FSMEventNotifier.hpp"
#include "romea_core_common/log/Logger.hpp"
#include "romea_core_localisation/dead_reckoning_tracking.hpp"
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
template<class Predictor>
std::unique_ptr<Predictor> make_kalman_predictor(
  std::shared_ptr<rclcpp::Node> & node,
  std::shared_ptr<core::Logger> logger,
  core::FSMEventCallback fsm_event_callback,
  const typename Predictor::ObservationAgeLimits & observation_age_limits)
{
  const core::localisation::DeadReckoningLimits dead_reckoning_limits(
    core::durationFromSecond(get_predictor_maximal_dead_reckoning_elapsed_time(node)),
    get_predictor_maximal_dead_reckoning_travelled_distance(node));

  auto predictor = std::make_unique<Predictor>(dead_reckoning_limits, observation_age_limits);

  predictor->register_logger(std::move(logger));
  predictor->register_fsm_event_callback(std::move(fsm_event_callback));
  return predictor;
}

//-----------------------------------------------------------------------------
template<class Predictor>
std::unique_ptr<Predictor> make_kalman_predictor(
  std::shared_ptr<rclcpp::Node> & node,
  std::shared_ptr<core::Logger> logger,
  core::FSMEventCallback fsm_event_callback = nullptr)
{
  return make_kalman_predictor<Predictor>(
    node,
    std::move(logger),
    std::move(fsm_event_callback),
    typename Predictor::ObservationAgeLimits());
}

//-----------------------------------------------------------------------------
template<class Predictor>
std::unique_ptr<Predictor> make_kalman_predictor(std::shared_ptr<rclcpp::Node> & node)
{
  return make_kalman_predictor<Predictor>(node, make_topic_logger(node, "predictor"));
}

//-----------------------------------------------------------------------------
template<class Predictor>
std::unique_ptr<Predictor> make_particle_predictor(
  std::shared_ptr<rclcpp::Node> & node,
  std::shared_ptr<core::Logger> logger,
  core::FSMEventCallback fsm_event_callback,
  const typename Predictor::ObservationAgeLimits & observation_age_limits)
{
  const core::localisation::DeadReckoningLimits dead_reckoning_limits(
    core::durationFromSecond(get_predictor_maximal_dead_reckoning_elapsed_time(node)),
    get_predictor_maximal_dead_reckoning_travelled_distance(node));

  auto predictor = std::make_unique<Predictor>(
    get_filter_number_of_particles(node), dead_reckoning_limits, observation_age_limits);

  predictor->register_logger(std::move(logger));
  predictor->register_fsm_event_callback(std::move(fsm_event_callback));
  return predictor;
}

//-----------------------------------------------------------------------------
template<class Predictor>
std::unique_ptr<Predictor> make_particle_predictor(
  std::shared_ptr<rclcpp::Node> & node,
  std::shared_ptr<core::Logger> logger,
  core::FSMEventCallback fsm_event_callback = nullptr)
{
  return make_particle_predictor<Predictor>(
    node,
    std::move(logger),
    std::move(fsm_event_callback),
    typename Predictor::ObservationAgeLimits());
}

//-----------------------------------------------------------------------------
template<class Predictor>
std::unique_ptr<Predictor> make_particle_predictor(std::shared_ptr<rclcpp::Node> & node)
{
  return make_particle_predictor<Predictor>(node, make_topic_logger(node, "predictor"));
}

//-----------------------------------------------------------------------------
template<class Predictor, core::FilterType FilterType_>
std::unique_ptr<Predictor> make_predictor(
  std::shared_ptr<rclcpp::Node> & node,
  core::FSMEventCallback fsm_event_callback,
  const typename Predictor::ObservationAgeLimits & observation_age_limits)
{
  if constexpr (FilterType_ == core::KALMAN) {
    return make_kalman_predictor<Predictor>(
      node,
      make_topic_logger(node, "predictor"),
      std::move(fsm_event_callback),
      observation_age_limits);
  } else {
    return make_particle_predictor<Predictor>(
      node,
      make_topic_logger(node, "predictor"),
      std::move(fsm_event_callback),
      observation_age_limits);
  }
}

//-----------------------------------------------------------------------------
template<class Predictor, core::FilterType FilterType_>
std::unique_ptr<Predictor> make_predictor(
  std::shared_ptr<rclcpp::Node> & node, core::FSMEventCallback fsm_event_callback = nullptr)
{
  return make_predictor<Predictor, FilterType_>(
    node, std::move(fsm_event_callback), typename Predictor::ObservationAgeLimits());
}

//-----------------------------------------------------------------------------
template<class Filter>
std::unique_ptr<Filter> make_kalman_filter(std::shared_ptr<rclcpp::Node> node)
{
  return std::make_unique<Filter>(get_filter_state_pool_size(node));
}

//-----------------------------------------------------------------------------
template<class Filter>
std::unique_ptr<Filter> make_particle_filter(std::shared_ptr<rclcpp::Node> node)
{
  return std::make_unique<Filter>(
    get_filter_state_pool_size(node), get_filter_number_of_particles(node));
}

//-----------------------------------------------------------------------------
template<class Filter, core::FilterType FilterType_>
std::unique_ptr<Filter> make_filter(std::shared_ptr<rclcpp::Node> node)
{
  if constexpr (FilterType_ == core::KALMAN) {
    return make_kalman_filter<Filter>(node);
  } else {
    return make_particle_filter<Filter>(node);
  }
}

//-----------------------------------------------------------------------------
template<class Filter, class Predictor, core::FilterType FilterType_>
std::unique_ptr<Filter> make_filter(
  std::shared_ptr<rclcpp::Node> node,
  core::FSMEventCallback fsm_event_callback,
  const typename Predictor::ObservationAgeLimits & observation_age_limits)
{
  auto filter = make_filter<Filter, FilterType_>(node);
  auto predictor = make_predictor<Predictor, FilterType_>(
    node, std::move(fsm_event_callback), observation_age_limits);
  filter->register_predictor(std::move(predictor));
  return filter;
}

//-----------------------------------------------------------------------------
template<class Filter, class Predictor, core::FilterType FilterType_>
std::unique_ptr<Filter> make_filter(
  std::shared_ptr<rclcpp::Node> node, core::FSMEventCallback fsm_event_callback = nullptr)
{
  return make_filter<Filter, Predictor, FilterType_>(
    node, std::move(fsm_event_callback), typename Predictor::ObservationAgeLimits());
}

//-----------------------------------------------------------------------------
template<class MetaState>
std::unique_ptr<MetaState> make_kalman_meta_state(std::shared_ptr<rclcpp::Node> /*node*/)
{
  return std::make_unique<MetaState>();
}

//-----------------------------------------------------------------------------
template<class MetaState>
std::unique_ptr<MetaState> make_particle_meta_state(std::shared_ptr<rclcpp::Node> node)
{
  return std::make_unique<MetaState>(get_filter_number_of_particles(node));
}

//-----------------------------------------------------------------------------
template<class MetaState, core::FilterType FilterType_>
std::unique_ptr<MetaState> make_meta_state(std::shared_ptr<rclcpp::Node> node)
{
  if constexpr (FilterType_ == core::KALMAN) {
    return make_kalman_meta_state<MetaState>(node);
  } else {
    return make_particle_meta_state<MetaState>(node);
  }
}

//-----------------------------------------------------------------------------
template<class Converter>
std::unique_ptr<Converter> make_kalman_meta_state_to_results(std::shared_ptr<rclcpp::Node> /*node*/)
{
  return std::make_unique<Converter>();
}

//-----------------------------------------------------------------------------
template<class Converter>
std::unique_ptr<Converter> make_particle_meta_state_to_results(std::shared_ptr<rclcpp::Node> node)
{
  return std::make_unique<Converter>(get_filter_number_of_particles(node));
}

//-----------------------------------------------------------------------------
template<class Converter, core::FilterType FilterType_>
std::unique_ptr<Converter> make_meta_state_to_results(std::shared_ptr<rclcpp::Node> node)
{
  if constexpr (FilterType_ == core::KALMAN) {
    return make_kalman_meta_state_to_results<Converter>(node);
  } else {
    return make_particle_meta_state_to_results<Converter>(node);
  }
}

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__FACTORY_HPP_
