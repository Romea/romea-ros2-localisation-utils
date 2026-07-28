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

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__UPDATER_INTERFACES_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__UPDATER_INTERFACES_HPP_

// std
#include <functional>
#include <list>
#include <memory>
#include <string>
#include <utility>

// local
#include "romea_localisation_utils/filter/updater_interface.hpp"

namespace romea
{
namespace ros2
{
namespace localisation
{

class UpdaterInterfaces
{
private:
  using Builder = std::function<void(std::shared_ptr<rclcpp::Node>)>;

public:
  template<typename Interface>
  void add(
    const std::string & updater_name,
    const std::string & topic_name,
    typename Interface::UpdateCallback update_callback);

  void create(std::shared_ptr<rclcpp::Node> node)
  {
    for (auto & builder : builders_) {
      builder(node);
    }

    builders_.clear();
  }

private:
  std::list<std::unique_ptr<UpdaterInterfaceBase>> instances_;
  std::list<Builder> builders_;
};

//-----------------------------------------------------------------------------
template<typename Interface>
void UpdaterInterfaces::add(
  const std::string & updater_name,
  const std::string & topic_name,
  typename Interface::UpdateCallback update_callback)
{
  builders_.emplace_back(
    [this, updater_name, topic_name, update_callback = std::move(update_callback)](
      std::shared_ptr<rclcpp::Node> node) mutable {
      instances_.push_back(
        make_updater_interface<typename Interface::Updater, typename Interface::Msg>(
          node, topic_name, std::move(update_callback)));
      RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": started ");
    });
}

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__UPDATER_INTERFACES_HPP_
