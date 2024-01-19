// 
// PROS Arm Commercial License 
// Copyright 2023 帕亞科技 (PAIA-Tech) . All rights reserved.
// Primary Author: 陳麒麟(Kylin Chen)
// Contributor(s): 蘇文鈺(Alvin Su) ,
// This software and associated documentation files are proprietary to 帕亞科技 (PAIA-Tech), 
// and are not to be copied, reproduced, or disclosed to any third party, in whole or in part, without 
// express prior written permission. Use and modification of this software is permitted for licensed 
// users only, and must always remain proprietary to 帕亞科技 (PAIA-Tech).
//

/*
This code is a mock arm.
It will publish mock arm position and receive target position.
You could use this to test how to visualize arm in simulation environment.
*/

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <string>
#include <iostream>
#include <cstdio>

#include <sstream>
using namespace std::chrono_literals;

using std::cerr;
using std::cout;
using std::endl;
using std::exception;
using std::string;
using std::vector;
#define DEG_TO_RAD 0.0174532925
#define DELTA_ANGLE 1.0
static const char *arm_names[] = {"joint1", "joint2", "joint3"};
const size_t arm_names_size = sizeof(arm_names) / sizeof(arm_names[0]);


class MockArmPubNode : public rclcpp::Node
{
public:
  MockArmPubNode() : Node("mock_arm")
  {
    // Create a publisher that sends `sensor_msgs::msg::JointState` messages
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&MockArmPubNode::callback, this));

    // Initialize current and desired joint positions to 90 degrees
    current_positions_ = {90.0, 90.0, 90.0};
    desired_positions_ = {0.0, 0.0, 0.0};

    // Create a subscriber that listens for `trajectory_msgs::msg::JointTrajectory` messages
    subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory",
        10,
        std::bind(&MockArmPubNode::trajectory_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscriber_;
  std::vector<double> current_positions_;
  std::vector<double> desired_positions_;

  void callback()
  {

    auto message = sensor_msgs::msg::JointState();

    // Fill in the JointState data
    message.header.stamp = this->get_clock()->now();

    // Convert the C-style array to the appropriate type (probably std::vector<std::string>)
    message.name.assign(arm_names, arm_names + arm_names_size);
    message.position.resize(current_positions_.size());

    // Update current positions to gradually reach the desired positions
    for (size_t i = 0; i < current_positions_.size(); i++)
    {
      if (std::abs(desired_positions_[i] - current_positions_[i]) > 1.1)
      {
        current_positions_[i] += (desired_positions_[i] > current_positions_[i]) ? DELTA_ANGLE : -DELTA_ANGLE;
        message.position[i] = current_positions_[i] * DEG_TO_RAD;
      }
    }

    std::stringstream ss;
    ss << "Joint positions: ";
    for (const auto &pos : current_positions_)
    {
      ss << pos << ' ';
    }

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

    publisher_->publish(message);
  }

  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    // Update desired positions based on received JointTrajectory data
    // Assuming the order of joints in the received message matches our own
    if (msg->points.size() > 0 && msg->points[0].positions.size() == current_positions_.size())
    {
      desired_positions_ = msg->points[0].positions;
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockArmPubNode>());
  rclcpp::shutdown();
  return 0;
}
