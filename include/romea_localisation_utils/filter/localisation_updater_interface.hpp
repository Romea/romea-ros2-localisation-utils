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

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_UPDATER_INTERFACE_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_UPDATER_INTERFACE_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_localisation_utils/filter/localisation_parameters.hpp"
#include "romea_localisation_utils/filter/localisation_updater_interface_base.hpp"
#include "romea_localisation_utils/conversions/observation_conversions.hpp"


namespace romea
{
namespace ros2
{

template<typename Filter_, typename Updater_, typename Msg>
class LocalisationUpdaterInterface : public LocalisationUpdaterInterfaceBase
{
public:
  using Filter = Filter_;
  using Updater = Updater_;
  using Observation = typename Updater_::Observation;

public:
  LocalisationUpdaterInterface(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & topic_name);

  void process_message(typename Msg::ConstSharedPtr msg);

  void load_updater(std::unique_ptr<Updater> updater);

  void register_filter(std::shared_ptr<Filter> filter);

  bool heartbeat_callback(const core::Duration & duration) override;

  core::DiagnosticReport get_report() override;

private:
  std::shared_ptr<Filter> filter_;
  std::unique_ptr<Updater> updater_;
  std::shared_ptr<rclcpp::Subscription<Msg>> sub_;
};

//-----------------------------------------------------------------------------
template<typename Filter_, typename Updater_, typename Msg>
LocalisationUpdaterInterface<Filter_, Updater_, Msg>::LocalisationUpdaterInterface(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & topic_name)
: LocalisationUpdaterInterfaceBase(),
  filter_(nullptr),
  updater_(nullptr),
  sub_()
{
  auto callback = std::bind(
    &LocalisationUpdaterInterface::process_message,
    this, std::placeholders::_1);

  rclcpp::SubscriptionOptions options;
  options.callback_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  sub_ = node->create_subscription<Msg>(topic_name, best_effort(1), callback, options);
}


//-----------------------------------------------------------------------------
template<typename Filter_, typename Updater_, typename Msg>
void LocalisationUpdaterInterface<Filter_, Updater_, Msg>::load_updater(
  std::unique_ptr<Updater> updater)
{
  updater_.swap(updater);
}

//-----------------------------------------------------------------------------
template<typename Filter_, typename Updater_, typename Msg>
void LocalisationUpdaterInterface<Filter_, Updater_, Msg>::register_filter(
  std::shared_ptr<Filter> filter)
{
  filter_ = filter;
}

//-----------------------------------------------------------------------------
template<class Filter_, class Updater_, class Msg>
void LocalisationUpdaterInterface<Filter_, Updater_, Msg>::process_message(
  typename Msg::ConstSharedPtr msg)
{
  core::Duration duration = extract_duration(*msg);

  Observation observation = extract_obs<Observation>(*msg);

  auto updateFunction = std::bind(
    &Updater::update,
    updater_.get(),
    std::placeholders::_1,
    std::move(observation),
    std::placeholders::_2,
    std::placeholders::_3);

  filter_->process(duration, std::move(updateFunction));
}

//-----------------------------------------------------------------------------
template<class Filter_, class Updater_, class Msg>
bool LocalisationUpdaterInterface<Filter_, Updater_, Msg>::heartbeat_callback(
  const core::Duration & duration)
{
  return updater_->heartBeatCallback(duration);
}

//-----------------------------------------------------------------------------
template<class Filter_, class Updater_, class Msg>
core::DiagnosticReport LocalisationUpdaterInterface<Filter_, Updater_, Msg>::get_report()
{
  return updater_->getReport();
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


}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_UPDATER_INTERFACE_HPP_
