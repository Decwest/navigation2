// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "gtest/gtest.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include "nav2_mppi_controller/controller.hpp"

#include "utils/utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

class TestableMPPIController : public nav2_mppi_controller::MPPIController
{
public:
  using nav2_mppi_controller::MPPIController::getOptimizerSpeed;
  using nav2_mppi_controller::MPPIController::last_command_velocity_;
  using nav2_mppi_controller::MPPIController::resetLastCommandVelocity;
  using nav2_mppi_controller::MPPIController::use_last_command_velocity_;
};

TEST(ControllerStateTransitionTest, VelocitySourceSelection)
{
  TestableMPPIController controller;
  geometry_msgs::msg::Twist odometry;
  odometry.linear.x = 0.2;
  odometry.angular.z = 0.4;

  controller.use_last_command_velocity_ = false;
  EXPECT_DOUBLE_EQ(controller.getOptimizerSpeed(odometry).linear.x, odometry.linear.x);
  EXPECT_DOUBLE_EQ(controller.getOptimizerSpeed(odometry).angular.z, odometry.angular.z);

  controller.use_last_command_velocity_ = true;
  EXPECT_DOUBLE_EQ(controller.getOptimizerSpeed(odometry).linear.x, 0.0);
  EXPECT_DOUBLE_EQ(controller.getOptimizerSpeed(odometry).angular.z, 0.0);

  controller.last_command_velocity_.linear.x = 0.3;
  controller.last_command_velocity_.angular.z = -0.5;
  EXPECT_DOUBLE_EQ(
    controller.getOptimizerSpeed(odometry).linear.x,
    controller.last_command_velocity_.linear.x);
  EXPECT_DOUBLE_EQ(
    controller.getOptimizerSpeed(odometry).angular.z,
    controller.last_command_velocity_.angular.z);

  controller.resetLastCommandVelocity();
  EXPECT_DOUBLE_EQ(controller.last_command_velocity_.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(controller.last_command_velocity_.angular.z, 0.0);
}

// Tests basic transition from configure->active->process->deactive->cleanup

TEST(ControllerStateTransitionTest, ControllerNotFail)
{
  const bool visualize = true;
  TestCostmapSettings costmap_settings{};

  // Node Options
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params;
  setUpControllerParams(visualize, params);
  params.emplace_back(rclcpp::Parameter("dummy.use_last_command_velocity", true));
  options.parameter_overrides(params);

  auto node = getDummyNode(options);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  costmap_ros->setRobotFootprint(getDummySquareFootprint(0.01));

  auto controller = std::make_shared<TestableMPPIController>();
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> weak_ptr_node{node};
  controller->configure(weak_ptr_node, node->get_name(), tf_buffer, costmap_ros);
  controller->activate();

  TestPose start_pose = costmap_settings.getCenterPose();
  const double path_step = costmap_settings.resolution;
  TestPathSettings path_settings{start_pose, 8, path_step, path_step};

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);
  path.header.frame_id = costmap_ros->getGlobalFrameID();
  pose.header.frame_id = costmap_ros->getGlobalFrameID();

  controller->setPlan(path);

  EXPECT_TRUE(controller->use_last_command_velocity_);
  EXPECT_DOUBLE_EQ(controller->getOptimizerSpeed(velocity).linear.x, 0.0);

  geometry_msgs::msg::TwistStamped command;
  EXPECT_NO_THROW(command = controller->computeVelocityCommands(pose, velocity, {}));
  EXPECT_DOUBLE_EQ(controller->last_command_velocity_.linear.x, command.twist.linear.x);
  EXPECT_DOUBLE_EQ(controller->last_command_velocity_.angular.z, command.twist.angular.z);

  const auto command_before_new_plan = controller->last_command_velocity_;
  controller->setPlan(path);
  EXPECT_DOUBLE_EQ(
    controller->last_command_velocity_.linear.x, command_before_new_plan.linear.x);
  EXPECT_DOUBLE_EQ(
    controller->last_command_velocity_.angular.z, command_before_new_plan.angular.z);

  controller->setSpeedLimit(0.5, true);
  controller->setSpeedLimit(0.5, false);
  controller->setSpeedLimit(1.0, true);

  controller->reset();
  EXPECT_DOUBLE_EQ(controller->last_command_velocity_.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(controller->last_command_velocity_.angular.z, 0.0);

  controller->last_command_velocity_.linear.x = 0.3;
  controller->last_command_velocity_.angular.z = -0.5;
  controller->deactivate();
  EXPECT_DOUBLE_EQ(controller->last_command_velocity_.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(controller->last_command_velocity_.angular.z, 0.0);
  controller->cleanup();
}
