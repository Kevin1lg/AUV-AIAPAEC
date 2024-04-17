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

        # Declare the parameter
        self.declare_parameter('tp1', [0.0])
        self.declare_parameter('tp2', [0.0])
        self.declare_parameter('tp3', [0.0])
        self.declare_parameter('t', 0.0)

        # Get the parameter
        self.tp1_ = self.get_parameter('tp1').get_parameter_value()
        self.tp2_ = self.get_parameter('tp2').get_parameter_value()
        self.tp3_ = self.get_parameter('tp3').get_parameter_value()
        t_ = self.get_parameter('t').get_parameter_value()

        self.number_timer_ = self.create_timer(t_.double_value, self.publish_data)# 0.01 second 

        self.get_logger().info("Test data has been started.")

    def publish_data(self):
        msg1 = Float64()
        msg2 = Float64()
        msg3 = Float64()

        ## Forces to thrusters in Newton, you can to add more values in the array

        datos1 = self.tp1_.double_array_value
        datos2 = self.tp2_.double_array_value
        datos3 = self.tp3_.double_array_value
        

        if self.counter_ <= len(datos1)-2:
            msg1.data = datos1[self.counter_]
            msg2.data = datos2[self.counter_]
            msg3.data = datos3[self.counter_]
            self.counter_ = self.counter_ + 1
        else:
            msg1.data = datos1[self.counter_]
            msg2.data = datos2[self.counter_]
            msg3.data = datos3[self.counter_]
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