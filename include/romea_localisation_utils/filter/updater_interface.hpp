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

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__UPDATER_INTERFACE_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__UPDATER_INTERFACE_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_core_filtering/filter/filter_base.hpp"
#include "romea_localisation_utils/conversions/observation_conversions.hpp"
#include "romea_localisation_utils/filter/parameters.hpp"
#include "romea_localisation_utils/filter/updater_interface_base.hpp"

namespace romea
{
namespace ros2
{
namespace localisation
{

template<typename Filter_, typename Updater_, typename Msg>
class UpdaterInterface : public UpdaterInterfaceBase
{
public:
  using Filter = Filter_;
  using Updater = Updater_;
  using Observation = typename Updater_::Observation;

public:
  UpdaterInterface(std::shared_ptr<rclcpp::Node> node, const std::string & topic_name);

  void process_message(typename Msg::ConstSharedPtr msg);

  void load_updater(std::unique_ptr<Updater> updater);

  void register_filter(std::shared_ptr<Filter> filter);

  bool heartbeat_callback(const core::Duration & duration) override;

  core::DiagnosticReport get_report() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<Filter> filter_;
  std::unique_ptr<Updater> updater_;
  std::shared_ptr<rclcpp::Subscription<Msg>> sub_;
};

//-----------------------------------------------------------------------------
template<typename Filter_, typename Updater_, typename Msg>
UpdaterInterface<Filter_, Updater_, Msg>::UpdaterInterface(
  std::shared_ptr<rclcpp::Node> node, const std::string & topic_name)
: UpdaterInterfaceBase(), node_(node), filter_(nullptr), updater_(nullptr), sub_()
{
  auto callback = std::bind(&UpdaterInterface::process_message, this, std::placeholders::_1);

  rclcpp::SubscriptionOptions options;
  options.callback_group =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // callback_group_ =
  // node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  sub_ = node->create_subscription<Msg>(topic_name, best_effort(1), callback, options);
}

//-----------------------------------------------------------------------------
template<typename Filter_, typename Updater_, typename Msg>
void UpdaterInterface<Filter_, Updater_, Msg>::load_updater(std::unique_ptr<Updater> updater)
{
  updater_.swap(updater);
}

//-----------------------------------------------------------------------------
template<typename Filter_, typename Updater_, typename Msg>
void UpdaterInterface<Filter_, Updater_, Msg>::register_filter(std::shared_ptr<Filter> filter)
{
  filter_ = filter;
}

//-----------------------------------------------------------------------------
template<class Filter_, class Updater_, class Msg>
void UpdaterInterface<Filter_, Updater_, Msg>::process_message(typename Msg::ConstSharedPtr msg)
{
  core::Duration duration = romea::ros2::extract_duration(*msg);

  Observation observation = romea::ros2::extract_obs<Observation>(*msg);

  auto update_function = std::bind(
    &Updater::update,
    updater_.get(),
    std::placeholders::_1,
    std::move(observation),
    std::placeholders::_2,
    std::placeholders::_3);

  const auto process_status = filter_->process(duration, std::move(update_function));
  if (process_status == core::FilterProcessStatus::TOO_OLD) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Discard observation because it is too old");
  }
}

//-----------------------------------------------------------------------------
template<class Filter_, class Updater_, class Msg>
bool UpdaterInterface<Filter_, Updater_, Msg>::heartbeat_callback(const core::Duration & duration)
{
  return updater_->heart_beat_callback(duration);
}

//-----------------------------------------------------------------------------
template<class Filter_, class Updater_, class Msg>
core::DiagnosticReport UpdaterInterface<Filter_, Updater_, Msg>::get_report()
{
  return updater_->get_report();
}

//-----------------------------------------------------------------------------
template<typename UpdaterInterface>
std::unique_ptr<UpdaterInterface> make_updater_interface(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & topic_name,
  std::shared_ptr<typename UpdaterInterface::Filter> filter,
  std::unique_ptr<typename UpdaterInterface::Updater> updater)
{
  auto interface = std::make_unique<UpdaterInterface>(node, topic_name);
  interface->load_updater(std::move(updater));
  interface->register_filter(filter);
  return interface;
}

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__UPDATER_INTERFACE_HPP_
