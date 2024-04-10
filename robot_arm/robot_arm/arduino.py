#!/usr/bin/python3

import mediapipe as mp
import rclpy
from rclpy.node import Node
from robot_arm_msg.msg import ArmDists

# TODO: Convert values to degrees
class Arduino(Node):
    def __init__(self):
        super().__init__('ArmDists')
        self.subscription = self.create_subscription(ArmDists, '/arm_dists', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        self.serial_send(msg)

    def serial_send(self, arg):
        # write the code the arduino needs
        print(arg)


def main():
    # Arduino port
    #ser_port = rospy.get_param('port2')
    #ser = serial.Serial(ser_port, 9600)

    rclpy.init()
    arduino_subscriber = Arduino()
    rclpy.spin(arduino_subscriber)
    arduino_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


