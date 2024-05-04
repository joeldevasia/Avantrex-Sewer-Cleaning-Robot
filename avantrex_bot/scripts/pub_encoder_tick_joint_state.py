#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import time
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.left_encoder_count = 0
        self.left_joint_angle = 0.0
        self.right_encoder_count = 0
        self.right_joint_angle = 0.0
        self.total_ticks_per_rev = 146
        self.left_encoder_sub = self.create_subscription(Int32, '/left_encoder_tick', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Int32, '/right_encoder_tick', self.right_encoder_callback, 10)
        self.publisher_ = self.create_publisher(JointState, '/motors_response', 10)
        self.timer_ = self.create_timer(0.01, self.publish_joint_states)

    def left_encoder_callback(self, msg):
        self.left_encoder_count = msg.data
        self.left_joint_angle = float((2 * math.pi * self.left_encoder_count) / self.total_ticks_per_rev)

    def right_encoder_callback(self, msg):
        self.right_encoder_count = msg.data
        self.right_joint_angle = float((2 * math.pi * self.right_encoder_count) / self.total_ticks_per_rev)

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state_msg.position = [self.left_joint_angle, self.right_joint_angle] # Positions change over time
        joint_state_msg.velocity = [float(0.5), float(0.5)]
        joint_state_msg.effort = [float(0.5), float(0.5)]
        self.publisher_.publish(joint_state_msg)
        print("Left count: ", self.left_encoder_count, "Right count: ", self.right_encoder_count)

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
