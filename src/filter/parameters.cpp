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
#include <memory>
#include <string>

// romea
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_localisation_utils/filter/parameters.hpp"

// #include <ros/file_log.h>

namespace
{

const char PREDICTOR_MAXIMAL_DEAD_RECKONING_TRAVELLED_DISTANCE_PARAM_NAME[] =
  "predictor.maximal_dead_recknoning_travelled_distance";
const char PREDICTOR_MAXIMAL_DEAD_RECKONING_ELAPSED_TIME_PARAM_NAME[] =
  "predictor.maximal_dead_recknoning_elapsed_time";

const char FILTER_NUMBER_OF_PARTICLES_PARAM_NAME[] = "filter.number_of_particles";
const char FILTER_STATE_POOL_SIZE_PARAM_NAME[] = "filter.state_pool_size";

const char UPDATER_TRIGGER_PARAM_NAME[] = "trigger";
const char UPDATER_MINIMAL_RATE_PARAM_NAME[] = "minimal_rate";
const char UPDATER_MAHALANOBIS_DISTANCE_REJECTION_THRESHOLD_PARAM_NAME[] =
  "mahalanobis_distance_rejection_threshold";

}  // namespace

namespace romea
{
namespace ros2
{
namespace localisation
{

//-----------------------------------------------------------------------------
void declare_predictor_parameters(
  std::shared_ptr<rclcpp::Node> node,
  const double & defaul_maximal_dead_reckoning_travelled_distance,
  const double & maximal_dead_reckoning_elapsed_time)
{
  declare_predictor_maximal_dead_reckoning_travelled_distance(
    node, defaul_maximal_dead_reckoning_travelled_distance);
  declare_predictor_maximal_dead_reckoning_elapsed_time(node, maximal_dead_reckoning_elapsed_time);
}

//-----------------------------------------------------------------------------
void declare_predictor_maximal_dead_reckoning_travelled_distance(
  std::shared_ptr<rclcpp::Node> node, const double & default_value)
{
  declare_parameter_with_default<double>(
    node, PREDICTOR_MAXIMAL_DEAD_RECKONING_TRAVELLED_DISTANCE_PARAM_NAME, default_value);
}

//-----------------------------------------------------------------------------
double get_predictor_maximal_dead_reckoning_travelled_distance(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(
    node, PREDICTOR_MAXIMAL_DEAD_RECKONING_TRAVELLED_DISTANCE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_predictor_maximal_dead_reckoning_elapsed_time(
  std::shared_ptr<rclcpp::Node> node, const double & default_value)
{
  declare_parameter_with_default<double>(
    node, PREDICTOR_MAXIMAL_DEAD_RECKONING_ELAPSED_TIME_PARAM_NAME, default_value);
}

//-----------------------------------------------------------------------------
double get_predictor_maximal_dead_reckoning_elapsed_time(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(node, PREDICTOR_MAXIMAL_DEAD_RECKONING_ELAPSED_TIME_PARAM_NAME);
}
//-----------------------------------------------------------------------------
void declare_kalman_filter_parameters(std::shared_ptr<rclcpp::Node> node)
{
  declare_filter_state_pool_size(node);
}

//-----------------------------------------------------------------------------
void declare_particle_filter_parameters(std::shared_ptr<rclcpp::Node> node)
{
  declare_filter_state_pool_size(node);
  declare_filter_number_of_particles(node);
}

//-----------------------------------------------------------------------------
void declare_filter_number_of_particles(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, FILTER_NUMBER_OF_PARTICLES_PARAM_NAME);
}

//-----------------------------------------------------------------------------
size_t get_filter_number_of_particles(std::shared_ptr<rclcpp::Node> node)
{
  return static_cast<size_t>(get_parameter<int>(node, FILTER_NUMBER_OF_PARTICLES_PARAM_NAME));
}

//-----------------------------------------------------------------------------
void declare_filter_state_pool_size(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, FILTER_STATE_POOL_SIZE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
size_t get_filter_state_pool_size(std::shared_ptr<rclcpp::Node> node)
{
  return static_cast<size_t>(get_parameter<int>(node, FILTER_STATE_POOL_SIZE_PARAM_NAME));
}

//-----------------------------------------------------------------------------
void declare_proprioceptive_updater_parameters(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const unsigned int & default_minimal_rate)
{
  // declare_updater_topic_name(node, updater_name);
  declare_updater_minimal_rate(node, updater_name, default_minimal_rate);
}

//-----------------------------------------------------------------------------
void declare_exteroceptive_updater_parameters(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const unsigned int & default_minimal_rate,
  const std::string & default_trigger_mode,
  const double & default_mahalanobis_distance_rejection_threshold)
{
  // declare_updater_topic_name(node, updater_name);
  declare_updater_minimal_rate(node, updater_name, default_minimal_rate);
  declare_updater_trigger_mode(node, updater_name, default_trigger_mode);
  declare_updater_mahalanobis_distance_rejection_threshold(
    node, updater_name, default_mahalanobis_distance_rejection_threshold);
}

//-----------------------------------------------------------------------------
void declare_updater_trigger_mode(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const std::string & default_value)
{
  declare_parameter_with_default<std::string>(
    node, updater_name, UPDATER_TRIGGER_PARAM_NAME, default_value);
}

//-----------------------------------------------------------------------------
std::string get_updater_trigger_mode(
  std::shared_ptr<rclcpp::Node> node, const std::string & updater_name)
{
  return get_parameter<std::string>(node, updater_name, UPDATER_TRIGGER_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_updater_minimal_rate(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const unsigned int & default_value)
{
  declare_parameter_with_default<int>(
    node, updater_name, UPDATER_MINIMAL_RATE_PARAM_NAME, default_value);
}

//-----------------------------------------------------------------------------
unsigned int get_updater_minimal_rate(
  std::shared_ptr<rclcpp::Node> node, const std::string & updater_name)
{
  int minimal_rate = get_parameter<int>(node, updater_name, UPDATER_MINIMAL_RATE_PARAM_NAME);

  if (minimal_rate < 0) {
    throw(std::runtime_error("Invalid minimal rate for updater " + updater_name));
  }

  return static_cast<unsigned int>(minimal_rate);
}

//-----------------------------------------------------------------------------
void declare_updater_mahalanobis_distance_rejection_threshold(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const double & default_value)
{
  declare_parameter_with_default<double>(
    node, updater_name, UPDATER_MAHALANOBIS_DISTANCE_REJECTION_THRESHOLD_PARAM_NAME, default_value);
}

//-----------------------------------------------------------------------------
double get_updater_mahalanobis_distance_rejection_threshold(
  std::shared_ptr<rclcpp::Node> node, std::string updater_name)
{
  return get_parameter<double>(
    node, updater_name, UPDATER_MAHALANOBIS_DISTANCE_REJECTION_THRESHOLD_PARAM_NAME);
}

}  // namespace localisation
}  // namespace ros2
}  // namespace romea
