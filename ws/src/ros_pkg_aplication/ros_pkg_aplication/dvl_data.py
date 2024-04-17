#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

class DataDvlNode(Node):
    def __init__(self):
        super().__init__("dvl_data")

        self.data_publisher = self.create_publisher(Vector3,'Velocidad_lineal_DVL', 10)

        self.number_subscriber = self.create_subscription(Vector3,'/aiapaec/dvl', self.callback_operation1, 10)

        self.get_logger().info("Data DVL has been started.")

        self.msg_velocidad_lineal = Vector3()

        self.timer = self.create_timer(0.1, self.timer_callback) # 0.1 second

    def callback_operation1(self, msg):
        self.msg_velocidad_lineal = msg

    def timer_callback(self):
        self.data_publisher.publish(self.msg_velocidad_lineal) # m/s

def main(args=None):
    rclpy.init(args=args)
    node = DataDvlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
