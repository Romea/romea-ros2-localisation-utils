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
#include <limits>
#include <memory>
#include <string>

// gtest
#include "gtest/gtest.h"

// romea
#include "../test/test_utils.hpp"
#include "../test/test_helper.h"
#include "romea_localisation_utils/filter/parameters.hpp"

using namespace romea::ros2::localisation;  // NOLINT

//-----------------------------------------------------------------------------
class TestLocalisationFilterParams : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions no;
    no.arguments(
      {"--ros-args", "--params-file", std::string(
          TEST_DIR) + "/test_localisation_parameters.yaml"});

    std::cout << "  loc file" << std::string(TEST_DIR) + "/test_localisation_parameters.yaml" <<
      std::endl;
    node = std::make_shared<rclcpp::Node>("test_localisation_parameters", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetPredictorMaximalDeadReckoningTravelledDistance)
{
  declare_predictor_maximal_dead_reckoning_travelled_distance(
    node, std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(get_predictor_maximal_dead_reckoning_travelled_distance(node), 10.0);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetPredictorMaximalDeadReckoningElapsedTime)
{
  declare_predictor_maximal_dead_reckoning_elapsed_time(
    node, std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(get_predictor_maximal_dead_reckoning_elapsed_time(node), 3.0);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetPredictorMaximalCircularErrorProbable)
{
  declare_predictor_maximal_circular_error_probable(
    node, std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(get_predictor_maximal_circular_error_probable(node), 0.2);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetFilterNumberOfParticles)
{
  declare_filter_number_of_particles(node);
  EXPECT_EQ(get_filter_number_of_particles(node), 200u);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetFilterStatePoolSize)
{
  declare_filter_state_pool_size(node);
  EXPECT_EQ(get_filter_state_pool_size(node), 1000u);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterTriggerMode)
{
  declare_updater_trigger_mode(node, "position_updater", "once");
  EXPECT_EQ(get_updater_trigger_mode(node, "position_updater"), "always");
}

// //-----------------------------------------------------------------------------
// TEST_F(TestLocalisationFilterParams, checkGetUpdaterTopicName)
// {
//   romea::declare_updater_topic_name(node, "pose_updater");
//   EXPECT_EQ(romea::get_updater_topic_name(node, "pose_updater"), "pose");
// }

// //-----------------------------------------------------------------------------
// TEST_F(TestLocalisationFilterParams, checkGetUpdaterEmptyTopicName)
// {
//   romea::declare_updater_topic_name(node, "foo");
//   EXPECT_EQ(romea::get_updater_topic_name(node, "foo"), "");
// }

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterMinimalRate)
{
  declare_updater_minimal_rate(node, "angular_speed_updater", 20u);
  EXPECT_EQ(get_updater_minimal_rate(node, "angular_speed_updater"), 10u);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterEmptyMinimalRate)
{
  declare_updater_minimal_rate(node, "bar", 10u);
  EXPECT_EQ(get_updater_minimal_rate(node, "bar"), 10u);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterMahalanobisDistance)
{
  declare_updater_mahalanobis_distance_rejection_threshold(
    node, "position_updater", std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(
    get_updater_mahalanobis_distance_rejection_threshold(node, "position_updater"), 3);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterEmptyMahalanobisDistance)
{
  declare_updater_mahalanobis_distance_rejection_threshold(node, "bar", 3);
  EXPECT_DOUBLE_EQ(get_updater_mahalanobis_distance_rejection_threshold(node, "bar"), 3);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
