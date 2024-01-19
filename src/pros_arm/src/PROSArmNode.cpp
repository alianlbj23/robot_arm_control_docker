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


#include "pros_library/pros_node.hpp"
#include <memory>
#include "serial/serial.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <string>
#include <iostream>
#include <cstdio>
#include <random>

#include <sstream>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

using std::cerr;
using std::cout;
using std::endl;
using std::exception;
using namespace serial;
using std::string;
using std::vector;

#define DEG_TO_RAD 0.0174532925
static const char *arm_names[] = {"base_joint", "torso_joint", "upperarm_joint", "elbow_joint", "forearm_joint"};
const size_t arm_names_size = sizeof(arm_names) / sizeof(arm_names[0]);

class PROSArmNode : public pros_library::ProsNode
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscriber_;
    std::vector<double> current_positions_;
    std::vector<double> desired_positions_;
    serial::Serial *my_serial;

public:
    PROSArmNode(const std::string &node_name, bool intra_process_comms = false)
        : pros_library::ProsNode::ProsNode(node_name, intra_process_comms)
    {
        RCLCPP_INFO(this->get_logger(), "Node Constructor");
        this->declare_parameter<string>("serial", "/dev/ttyUSB0");
        this->configure();
        this->activate();
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "On configure");

        // TODO: configure
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&PROSArmNode::callback, this));

        // Initialize current and desired joint positions to 90 degrees
        current_positions_ = {90.0, 90.0, 90.0, 90.0, 90.0};
        desired_positions_ = {90.0, 90.0, 90.0, 90.0, 90.0};

        // Create a subscriber that listens for `trajectory_msgs::msg::JointTrajectory` messages
        subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "joint_trajectory",
            10,
            std::bind(&PROSArmNode::trajectory_callback, this, std::placeholders::_1));
        string serial_port;
        this->get_parameter("serial", serial_port);
        this->my_serial = new serial::Serial(serial_port, 115200, serial::Timeout::simpleTimeout(1000));
        cout << "Is the serial port open?";
        if (this->my_serial->isOpen())
            cout << " Yes." << endl;
        else
            cout << " No." << endl;

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "On active");

        // TODO: What will be executed when node activating.

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "On deactive");

        // TODO: Turn off/Reset the operations when activating.

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "On cleanup");

        // TODO: Clean all the variables.

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    void callback()
    {
        // Create a new JointState message
        auto message = sensor_msgs::msg::JointState();

        // Fill in the JointState data
        message.header.stamp = this->get_clock()->now();
        message.name.assign(arm_names, arm_names + arm_names_size);

        string json_str = this->my_serial->readline();
        // 1. parse json_str and create json object.
        json j;
        try
        {
            // RCLCPP_INFO(this->get_logger(), json_str.c_str());

            j = json::parse(json_str);

        }
        catch (json::parse_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", json_str);
            return; // Exit the function if parsing fails
        }
        // 2. get position data in key: "servo_current_angles" and put into message object.
        if (j.contains("servo_current_angles") && j["servo_current_angles"].is_array())
        {
            for (const auto &angle : j["servo_current_angles"])
            {
                message.position.push_back(angle.get<double>() * DEG_TO_RAD);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "servo_current_angles not found or not an array in JSON");
        }
        // 3. publish  joint_states

        publisher_->publish(message);

        // 4. Log joint_states
        std::stringstream ss;
        ss << "Joint positions: ";
        for (const auto &pos : message.position)
        {
            ss << pos << ' ';
        }
        // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // Update desired positions based on received JointTrajectory data
        // Assuming the order of joints in the received message matches our own
        RCLCPP_INFO(this->get_logger(), "trajectory_callback");
        std::stringstream ss;
        ss << "Receive JointTrajectory: ";
        for (size_t i = 0; i < msg->points[0].positions.size(); i++)
        {
            ss <<  msg->points[0].positions[i] << ' ';
            desired_positions_[i] = msg->points[0].positions[i];
        }
        RCLCPP_INFO(this->get_logger(),  ss.str().c_str());
        // send control signal to serial
        json j;
        j["servo_target_angles"] = desired_positions_;


        string output_str = j.dump();
        RCLCPP_INFO(this->get_logger(), output_str.c_str());
        this->my_serial->write(output_str + "\n");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    pros_library::run(std::make_shared<PROSArmNode>("pros_arm"));
    return 0;
}
