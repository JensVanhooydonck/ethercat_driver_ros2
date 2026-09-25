// Copyright 2023 ICUBE Laboratory, University of Strasbourg
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

#include <gtest/gtest.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

// One joint on mock_components/GenericSystem: ros2_control_test_assets::minimal_robot_urdf
// uses the test_components hardware plugins, which throw std::bad_alloc on this
// Jazzy install before the controller under test is ever loaded.
static const char kMockUrdf[] = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <ros2_control name="TestSystem" type="system">
    <hardware><plugin>mock_components/GenericSystem</plugin></hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
)";

TEST(TestLoadCiA402Controller, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(
      kMockUrdf, std::make_shared<rclcpp::Clock>(),
      rclcpp::get_logger("test_controller_manager")),
    executor, "test_controller_manager");

  ASSERT_NO_THROW(
    cm.load_controller(
      "test_cia402_controller",
      "ethercat_controllers/CiA402Controller"));

  rclcpp::shutdown();
}
