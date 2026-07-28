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
#include <functional>
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_localisation_utils/conversions/observation_conversions.hpp"

namespace romea
{
namespace ros2
{
namespace localisation
{

class UpdaterInterfaceBase
{
public:
  virtual ~UpdaterInterfaceBase() = default;
};

template<typename Updater_, typename Msg_>
class UpdaterInterface : public UpdaterInterfaceBase
{
public:
  using Updater = Updater_;
  using Msg = Msg_;
  using Observation = typename Updater_::Observation;
  using UpdateCallback = std::function<void(const core::Duration &, const Observation &)>;

public:
  UpdaterInterface(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & topic_name,
    UpdateCallback update_callback);

  void process_message(typename Msg::ConstSharedPtr msg);

private:
  std::shared_ptr<rclcpp::Node> node_;
  UpdateCallback update_callback_;
  std::shared_ptr<rclcpp::Subscription<Msg_>> sub_;
};

//-----------------------------------------------------------------------------
template<typename Updater_, typename Msg_>
UpdaterInterface<Updater_, Msg_>::UpdaterInterface(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & topic_name,
  UpdateCallback update_callback)
: UpdaterInterfaceBase(),
  node_(std::move(node)),
  update_callback_(std::move(update_callback)),
  sub_()
{
  auto callback = std::bind(&UpdaterInterface::process_message, this, std::placeholders::_1);

  rclcpp::SubscriptionOptions options;
  options.callback_group =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  sub_ = node_->create_subscription<Msg_>(topic_name, best_effort(1), callback, options);
}

//-----------------------------------------------------------------------------
template<typename Updater_, typename Msg_>
void UpdaterInterface<Updater_, Msg_>::process_message(
  typename Msg_::ConstSharedPtr msg)
{
  update_callback_(
    romea::ros2::extract_duration(*msg),
    romea::ros2::extract_obs<Observation>(*msg));
}

//-----------------------------------------------------------------------------
template<typename Updater, typename Msg>
std::unique_ptr<UpdaterInterface<Updater, Msg>> make_updater_interface(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & topic_name,
  typename UpdaterInterface<Updater, Msg>::UpdateCallback update_callback)
{
  return std::make_unique<UpdaterInterface<Updater, Msg>>(
    std::move(node), topic_name, std::move(update_callback));
}

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__UPDATER_INTERFACE_HPP_
