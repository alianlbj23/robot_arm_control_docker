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
#include <sstream>
#include <nlohmann/json.hpp>
#include <stdlib.h>
#include <time.h>

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

class RandomTargetArmNode : public pros_library::ProsNode
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_create_target;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscriber_;
    std::vector<double> current_positions_;
    std::vector<double> desired_positions_;
    serial::Serial *my_serial;

public:
    RandomTargetArmNode(const std::string &node_name, bool intra_process_comms = false)
        : pros_library::ProsNode::ProsNode(node_name, intra_process_comms)
    {
        RCLCPP_INFO(this->get_logger(), "Node Constructor");
        this->declare_parameter<string>("serial", "/dev/ttyUSB1");
        this->configure();
        this->activate();
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "On configure");

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&RandomTargetArmNode::callback, this));
        timer_create_target = this->create_wall_timer(10000ms, std::bind(&RandomTargetArmNode::create_target_callback, this));

        //  Initialize current and desired joint positions to 90 degrees
        current_positions_ = std::vector<double>(5, 90.0);
        desired_positions_ = std::vector<double>(5, 10.0);

        srand(time(NULL));

        // Generate a random number between 0 and 180
        string serial_port;
        this->get_parameter("serial", serial_port);
        this->my_serial = new serial::Serial(serial_port, 115200, serial::Timeout::simpleTimeout(1000));
        cout << "Is the serial port open?";
        if (this->my_serial->isOpen())
        {
            cout << " Yes." << endl;
    
        }
        else
        {
            cout << " No." << endl;
        }

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "On active");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "On deactive");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "On cleanup");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    void callback()
    {
        // Create a new JointState message
        auto message = sensor_msgs::msg::JointState();

        // Fill in the JointState data
        message.header.stamp = this->get_clock()->now();
        message.name.assign(arm_names, arm_names + arm_names_size);
        message.position.resize(current_positions_.size());
        string json_str = this->my_serial->readline();
        // 1. parse json_str and create json object.
        json j;
        try
        {
            j = json::parse(json_str);
            RCLCPP_DEBUG(this->get_logger(), json_str.c_str());
        }
        catch (json::parse_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", json_str);
            return; // Exit the function if parsing fails
        }
        // 2. TODO get position data in key: "servo_current_angles" and put into message object.
        if (j.contains("servo_current_angles") && j["servo_current_angles"].is_array())
        {
            current_positions_.clear();
            for (const auto &angle : j["servo_current_angles"])
            {
                current_positions_.push_back(angle.get<double>());
                message.position.push_back(angle.get<double>() * DEG_TO_RAD);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "servo_current_angles not found or not an array in JSON");
        }
        // 3.  publish current joint_states

        publisher_->publish(message);

        
        // send json string to esp32

        // 5. Log joint_states
        std::stringstream ss;
        ss << "Joint positions: ";
        for (const auto &pos : current_positions_)
        {
            ss << pos << ' ';
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }
    void create_target_callback()
    {

        for (size_t i = 0; i < desired_positions_.size(); ++i)
        {
            desired_positions_[i] = rand() % 181;
        }
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
    pros_library::run(std::make_shared<RandomTargetArmNode>("pros_arm"));
    return 0;
}
