#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64

class DataPruebaNode(Node):
    def __init__(self):
        super().__init__("test_data")
        self.counter_ = 0

        self.number_publisher1_ = self.create_publisher(Float64,"/thruster1_joint/cmd_thrust",10)
        self.number_publisher2_ = self.create_publisher(Float64,"/thruster2_joint/cmd_thrust",10)
        self.number_publisher3_ = self.create_publisher(Float64,"/thruster3_joint/cmd_thrust",10)

        self.number_timer_ = self.create_timer(0.01, self.publish_data)# 0.01 second 

        self.get_logger().info("Test data has been started.")

    def publish_data(self):
        msg1 = Float64()
        msg2 = Float64()
        msg3 = Float64()

        ## Forces to thrusters in Newton, you can to add more values in the array

        datos1 = np.array([30.0, 30.0, 30.0, 30.0, 30.0, 30.0]) # right thruster TP1 
        datos2 = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0]) # left thruster TP2 
        datos3 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # center thruster TP3

        if self.counter_ <= len(datos1)-1:
            msg1.data = datos1[self.counter_]
            msg2.data = datos2[self.counter_]
            msg3.data = datos3[self.counter_]
            self.counter_ = self.counter_ + 1
        else:
            msg1.data = datos1[self.counter_-1]
            msg2.data = datos2[self.counter_-1]
            msg3.data = datos3[self.counter_-1]
            self.counter_ = 0

        self.number_publisher1_.publish(msg1)
        self.number_publisher2_.publish(msg2)
        self.number_publisher3_.publish(msg3)        

def main(args=None):
    rclpy.init(args=args)
    node = DataPruebaNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()