#!/usr/bin/env python3
import rclpy
import sys
# import cv2
# import math
# import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
# from scipy.spatial.transform import Rotation as R


class publish_imu(Node):

    def __init__(self):

        super().__init__("imu_data_publisher") 
        print("Starting IMU Data Publisher Node...")
        self.imu_data = None

        self.bno055_imu_sub = self.create_subscription(
            String, "/bno055/imu", self.bno055_imu_cb, 10
        )
        
        self.imu_pub = self.create_publisher(String, "/imu", 10)

        publish_rate = 0.2  
        # self.timer = self.create_timer(
        #     publish_rate, self.publish_imu_data
        # )  

    def bno055_imu_cb(self, data):
        self.imu_data = data.data
        print(self.imu_data)
    
    def publish_imu_data(self):
        msg = String()
        msg.data = self.imu_data
        self.imu_pub.publish(msg)
       


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