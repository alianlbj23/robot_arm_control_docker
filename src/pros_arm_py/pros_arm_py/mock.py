#
# PROS Arm Commercial License
# Copyright 2023 帕亞科技 (PAIA-Tech) . All rights reserved.
# Primary Author: 陳麒麟(Kylin Chen)
# Contributor(s): 蘇文鈺(Alvin Su),
# This software and associated documentation files are proprietary to 帕亞科技 (PAIA-Tech),
# and are not to be copied, reproduced, or disclosed to any third party, in whole or in part, without
# express prior written permission. Use and modification of this software is permitted for licensed
# users only, and must always remain proprietary to 帕亞科技 (PAIA-Tech).
#

# This code is a mock arm.
# It will publish mock arm position and receive target position.
# You could use this to test how to visualize arm in simulation environment.

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import math
from builtin_interfaces.msg import Time

DEG_TO_RAD = 0.0174532925
RAD_TO_DEG = 180.0 / math.pi  # Define the radians to degrees conversion factor
DELTA_ANGLE = 1.0
ARM_NAMES = ["joint1", "joint2", "joint3"]


class MockArmPubNode(Node):
    def __init__(self):
        super().__init__('mock_arm')
        # Create a publisher that sends `sensor_msgs/msg/JointState` messages
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # Create a timer with a callback to be called at a regular interval
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        
        # Initialize current and desired joint positions to 90 degrees
        self.current_positions_ = [90.0, 90.0, 90.0]
        self.desired_positions_ = [0.0, 0.0, 0.0]
        
        # Create a subscriber that listens for `trajectory_msgs/msg/JointTrajectory` messages
        self.subscriber_ = self.create_subscription(
            JointTrajectoryPoint,
            'joint_trajectory_point',
            self.trajectory_callback,
            10)

    def timer_callback(self):
        message = JointState()
        # Fill in the JointState data
        now = self.get_clock().now()
        message.header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
        message.name = ARM_NAMES
        message.position = [0.0] * len(ARM_NAMES)
        
        # Update current positions to gradually reach the desired positions
        for i, current in enumerate(self.current_positions_):
            if abs(self.desired_positions_[i] - current) > 1.1:
                direction = 1 if self.desired_positions_[i] > current else -1
                self.current_positions_[i] += direction * DELTA_ANGLE
                message.position[i] = self.current_positions_[i] * DEG_TO_RAD
        
        self.get_logger().info('Joint positions: ' + ' '.join(str(pos) for pos in self.current_positions_))
        
        self.publisher_.publish(message)

    def trajectory_callback(self, msg):
        # Update desired positions based on received JointTrajectoryPoint data
        if len(msg.positions) == len(self.current_positions_):
            self.desired_positions_ = [p * RAD_TO_DEG for p in msg.positions]

def main(args=None):
    rclpy.init(args=args)
    mock_arm_pub_node = MockArmPubNode()
    rclpy.spin(mock_arm_pub_node)
    mock_arm_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
