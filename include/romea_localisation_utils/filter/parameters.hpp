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

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__PARAMETERS_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__PARAMETERS_HPP_

// std
#include <memory>
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_core_filtering/filter/type.hpp"

namespace romea {
namespace ros2 {
namespace localisation {

void declare_predictor_parameters(
    std::shared_ptr<rclcpp::Node> node,
    const double& defaul_maximal_dead_reckoning_travelled_distance,
    const double& maximal_dead_reckoning_elapsed_time,
    const double& maximal_circular_error_probable);

void declare_predictor_maximal_dead_reckoning_travelled_distance(
    std::shared_ptr<rclcpp::Node> node, const double& default_value);

double get_predictor_maximal_dead_reckoning_travelled_distance(
    std::shared_ptr<rclcpp::Node> node);

void declare_predictor_maximal_dead_reckoning_elapsed_time(
    std::shared_ptr<rclcpp::Node> node, const double& default_value);

double get_predictor_maximal_dead_reckoning_elapsed_time(
    std::shared_ptr<rclcpp::Node> node);

void declare_predictor_maximal_circular_error_probable(
    std::shared_ptr<rclcpp::Node> node, const double& default_value);

double get_predictor_maximal_circular_error_probable(
    std::shared_ptr<rclcpp::Node> node);

void declare_kalman_filter_parameters(std::shared_ptr<rclcpp::Node> node);

void declare_particle_filter_parameters(std::shared_ptr<rclcpp::Node> node);

template <core::FilterType FilterType_>
void declare_filter_parameters(std::shared_ptr<rclcpp::Node> node) {
  if constexpr (FilterType_ == core::KALMAN) {
    return declare_kalman_filter_parameters(node);
  } else {
    return declare_particle_filter_parameters(node);
  }
}

void declare_filter_number_of_particles(std::shared_ptr<rclcpp::Node> node);

size_t get_filter_number_of_particles(std::shared_ptr<rclcpp::Node> node);

void declare_filter_state_pool_size(std::shared_ptr<rclcpp::Node> node);

size_t get_filter_state_pool_size(std::shared_ptr<rclcpp::Node> node);

// void declare_proprioceptive_updater_parameters(
//   std::shared_ptr<rclcpp::Node> node,
//   const std::string & updater_name);

void declare_proprioceptive_updater_parameters(
    std::shared_ptr<rclcpp::Node> node, const std::string& updater_name,
    const unsigned int& default_minimal_rate);

// void declare_exteroceptive_updater_parameters(
//   std::shared_ptr<rclcpp::Node> node,
//   const std::string & updater_name);

void declare_exteroceptive_updater_parameters(
    std::shared_ptr<rclcpp::Node> node, const std::string& updater_name,
    const unsigned int& default_minimal_rate,
    const std::string& default_trigger_mode,
    const double& default_mahalanobis_distance_rejection_threshold = 5.0);

// void declare_updater_trigger_mode(
//   std::shared_ptr<rclcpp::Node> node,
//   const std::string & updater_name);

void declare_updater_trigger_mode(std::shared_ptr<rclcpp::Node> node,
                                  const std::string& updater_name,
                                  const std::string& default_value);

std::string get_updater_trigger_mode(std::shared_ptr<rclcpp::Node> node,
                                     const std::string& updater_name);

// void declare_updater_topic_name(
//   std::shared_ptr<rclcpp::Node> node,
//   const std::string & updater_name);

// std::string get_updater_topic_name(
//   std::shared_ptr<rclcpp::Node> node,
//   const std::string & updater_name);

// void declare_updater_minimal_rate(
//   std::shared_ptr<rclcpp::Node> node,
//   const std::string & updater_name);

void declare_updater_minimal_rate(std::shared_ptr<rclcpp::Node> node,
                                  const std::string& updater_name,
                                  const unsigned int& minimal_rate);

unsigned int get_updater_minimal_rate(std::shared_ptr<rclcpp::Node> node,
                                      const std::string& updater_name);

// void declare_updater_mahalanobis_distance_rejection_threshold(
//   std::shared_ptr<rclcpp::Node> node,
//   const std::string & updater_name);

void declare_updater_mahalanobis_distance_rejection_threshold(
    std::shared_ptr<rclcpp::Node> node, const std::string& updater_name,
    const double& default_value);

double get_updater_mahalanobis_distance_rejection_threshold(
    std::shared_ptr<rclcpp::Node> node, std::string updater_name);

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__PARAMETERS_HPP_
