import json
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import std_msgs.msg

from serial import Serial

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')

        # Set up the serial connection
        self.serial = Serial('/dev/ttyUSB0', 115200,timeout=0)

        # Create a publisher for the serial data
        self.pub = self.create_publisher(std_msgs.msg.String, 'serial_data', qos_profile_sensor_data)

        # Set up a timer to read data from the serial device
        # 每 n 秒收一次資料
        self.tmr1 = self.create_timer(0.1, self.reader_callback)
        # self.tmr1 = self.create_timer(1, self.writer_callback)

    def reader_callback(self):
        
        # Read data from the serial device
        data = self.serial.readline()
        # Publish the data to the serial_data topic
        # msg = std_msgs.msg.String()
        # msg.data = data.decode('utf-8')
        # self.pub.publish(msg)
        # TODO ros logger
        print("reader: "+data.decode('utf-8'))
        # TODO collect imu and encoder data

    def writer_callback(self):
        ctrl_json={"servo_target_angles":[90,90,90,92,random.randint(10,100)]}
        ctrl_json = json.dumps(ctrl_json)+"\n"
        self.serial.write(ctrl_json.encode())
        # TODO ros logger
        print("writer:"+json.dumps(ctrl_json))






def main(args=None):
    rclpy.init(args=args)

    serial_reader = SerialReader()

    # rclpy.spin(serial_writer)
    rclpy.spin(serial_reader)

    serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
