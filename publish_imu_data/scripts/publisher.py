#!/usr/bin/env python3
import rclpy #type: ignore
import sys
# import cv2
import math
# import numpy as np
from rclpy.node import Node #type: ignore
from std_msgs.msg import String #type: ignore
from std_msgs.msg import Int32 #type: ignore
from sensor_msgs.msg import Imu #type: ignore
from transformations import quaternion_from_euler, euler_from_quaternion #type: ignore


class publish_imu(Node):

    def __init__(self):

        super().__init__("imu_data_publisher") 
        print("Starting IMU Data Publisher Node...")
        self.imu_data = None

        self.bno055_imu_sub = self.create_subscription(
            Imu, "/bno055/imu", self.bno055_imu_cb, 10
        )
        
        self.imu_pub = self.create_publisher(String, "/IMU_Data", 10)
        self.turn_robot = self.create_publisher(Int32, "/turn_robot", 10)

        publish_rate = 0.2  
        self.timer = self.create_timer(
            publish_rate, self.publish_imu_data
        )  

    def bno055_imu_cb(self, data):
        self.imu_data = data
    
    def publish_imu_data(self):
        msg = String()
        try:
            msg.data = str(self.imu_data.orientation.x)
            self.imu_pub.publish(msg)
            euler = euler_from_quaternion([self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w])
            # print("x: ", round(math.degrees(euler[0]),2), "y: ", round(math.degrees(euler[1]),2), "z: ", round(math.degrees(euler[2]),2))
            yaw = round(math.degrees(euler[0]),2)
            print("yaw: ", yaw)
            if abs(yaw)<=10:
                turn = Int32()
                turn.data = 0
                self.turn_robot.publish(turn) 
            elif yaw>0:
                turn = Int32()
                turn.data = 1
                self.turn_robot.publish(turn)
            else:
                turn = Int32()
                turn.data = -1
                self.turn_robot.publish(turn)
        except Exception as e:
            print(e)
       


##################### FUNCTION DEFINITION #######################


def main():
    """
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    """
    rclpy.init(args=sys.argv)  # initialisation
    node = rclpy.create_node("imu_publisher")  # creating ROS node
    node.get_logger().info("Node created: IMU Publisher")  # logging information
    publish_imu_class = publish_imu()  # creating a new object for class 'aruco_tf'
    rclpy.spin(publish_imu_class)  # spining on the object to make it alive in ROS 2 DDS
    publish_imu_class.destroy_node()  # destroy node after spin ends
    rclpy.shutdown()  # shutdown process


if __name__ == "__main__":
    """
    Description:    If the python interpreter is running that module (the source file) as the main program,
                    it sets the special __name__ variable to have a value “__main__”.
                    If this file is being imported from another module, __name__ will be set to the module's name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    """

    main()