#!/usr/bin/python3

from .robot_arm_IK import inverse_kinematics
import rclpy
from rclpy.node import Node
from robot_arm_msg.msg import ArmDists, CameraDists


class JointAngle(Node):
    def __init__(self):
        super().__init__('joint_angle_pub')
        self.subscription = self.create_subscription(CameraDists, '/camera_dists', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(ArmDists, '/arm_dists', 10)

    def listener_callback(self, msg):
        arm_array = inverse_kinematics(msg.x_dist, msg.y_dist, msg.z_dist, msg.r, msg.p, msg.y)
        message = ArmDists()
        message.one = arm_array[0]
        message.two = arm_array[1]
        message.three = arm_array[2]
        message.four = arm_array[3]
        message.five = arm_array[4]
        message.closed = msg.closed

        self.publisher_.publish(message)

def main():
    rclpy.init()
    joint_angle = JointAngle()
    rclpy.spin(joint_angle)
    joint_angle.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
