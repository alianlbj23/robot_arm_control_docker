#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import curses

SERIAL_DEV_DEFAULT = '/dev/ttyUSB0'  # replace with your default value

class KeyboardNode(Node):

    def __init__(self, stdscr):
        super().__init__('keyboard_listener')
        self.publisher_ = self.create_publisher(String, 'key_topic', 10)

        # You can initialize the serial connection here as per your original code if needed
        # self.serial = Serial(serial_port, 115200, timeout=0)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        # Define a publisher for JointTrajectoryPoint messages
        self.joint_trajectory_publisher_ = self.create_publisher(JointTrajectoryPoint, 'joint_trajectory_point', 10)

        self.stdscr = stdscr
        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)
        self.key_in_count = 0
        self.run()

    def joint_state_callback(self, msg: JointState):
        # This method is called when a JointState message is received
        # Process the received JointState message as needed
        self.get_logger().info(f'Received JointState: {msg.position}')

    def run(self):
        while rclpy.ok():
            c = self.stdscr.getch()
            self.key_in_count += 1
            self.print_key(c)

            if c == ord('w'):
                self.handle_key_w()
            elif c == ord('a'):
                self.handle_key_a()
            elif c == ord('s'):
                self.handle_key_s()
            elif c == ord('d'):
                self.handle_key_d()
            elif c == ord('i'):
                self.handle_key_i()
            elif c == ord('j'):
                self.handle_key_j()
            elif c == ord('k'):
                self.handle_key_k()
            elif c == ord('l'):
                self.handle_key_l()
            elif c == ord('u'):
                self.handle_key_u()
            elif c == ord('o'):
                self.handle_key_o()
            elif c == ord('y'):
                self.handle_key_y()
            elif c == ord('h'):
                self.handle_key_h()
            elif c == ord('q'):  # Exit on 'q'
                break
            # origin_string = self.serial.readline()
            # self.stdscr.move(3, 0)
            # self.stdscr.addstr(f"{self.key_in_count:5d} receive: {origin_string} ")




    def print_key(self, key):
        # Clear the screen
        self.stdscr.clear()

        self.stdscr.move(0, 0)
        # Print a string at the current cursor position
        self.stdscr.addstr(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

        self.stdscr.move(1, 0)
        self.get_logger().debug(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

    def handle_key_w(self):
        # Your action for the 'w' key here
        self.stdscr.addstr(f"car go forward")

        # self.stdscr.move(1, 0)
        pass
    
    def handle_key_a(self):
        # Your action for the 'a' key here
        self.stdscr.addstr(f"car turn left ")

        pass

    # Add methods for other keys similarly
    def handle_key_s(self):
        self.stdscr.addstr(f"car go backward")

        pass

    def handle_key_d(self):
        self.stdscr.addstr(f"car turn right")
        pass

    def handle_key_i(self):
        self.stdscr.addstr(f"arm rift up")
        pass

    def handle_key_j(self):
        self.stdscr.addstr(f"arm turn left")
        pass

    def handle_key_k(self):
        self.stdscr.addstr(f"arm rift down")
        pass

    def handle_key_l(self):
        self.stdscr.addstr(f"arm turn right")
        pass

    def handle_key_u(self):
        self.stdscr.addstr(f"arm j4 rotate left")
        pass

    def handle_key_o(self):
        self.stdscr.addstr(f"arm j4 rotate right")
        pass

    def handle_key_y(self):
        self.stdscr.addstr(f"arm catch!")
        msg = JointTrajectoryPoint()
        msg.positions = [1.0, 2.0, 3.0]  # Replace with actual desired positions
        msg.velocities = [0.1, 0.2, 0.3] # Replace with actual desired velocities
        # You can set other fields of the JointTrajectoryPoint message similarly.
        self.joint_trajectory_publisher_.publish(msg)

        # ctrl_json={"servo_target_angles":[90,90,90,90,100]}
        # ctrl_json = json.dumps(ctrl_json)+"\n"
        # self.serial.write(ctrl_json.encode())
        pass

    def handle_key_h(self):
        self.stdscr.addstr(f"arm release!")
        # ctrl_json={"servo_target_angles":[90,90,90,90,10]}
        # ctrl_json = json.dumps(ctrl_json)+"\n"
        # self.serial.write(ctrl_json.encode())
        pass

# ... Rest of your code, e.g. initializing rclpy and running the node

def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    try:
        node = KeyboardNode(stdscr)
    finally:
        curses.endwin()
        node.get_logger().info(f'Quit keyboard!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
