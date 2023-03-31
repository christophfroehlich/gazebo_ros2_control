// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("position_topic_test_node");

  auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  trajectory_msgs::msg::JointTrajectory commands;

  using namespace std::chrono_literals;
  
  std::vector<std::string> joint_names = {"slider_to_cart"};

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap
  point.positions.resize(joint_names.size());

  point.positions[0] = 0.0;

  trajectory_msgs::msg::JointTrajectoryPoint point2;
  point2.time_from_start = rclcpp::Duration::from_seconds(1.0);
  point2.positions.resize(joint_names.size());
  point2.positions[0] = -1.0;

  trajectory_msgs::msg::JointTrajectoryPoint point3;
  point3.time_from_start = rclcpp::Duration::from_seconds(2.0);
  point3.positions.resize(joint_names.size());
  point3.positions[0] = 1.0;

  trajectory_msgs::msg::JointTrajectoryPoint point4;
  point4.time_from_start = rclcpp::Duration::from_seconds(3.0);
  point4.positions.resize(joint_names.size());
  point4.positions[0] = 0.0;

  points.push_back(point);
  points.push_back(point2);
  points.push_back(point3);
  points.push_back(point4);

  commands.joint_names = joint_names;
  commands.points = points;

  publisher->publish(commands);
  rclcpp::shutdown();

  return 0;
}
