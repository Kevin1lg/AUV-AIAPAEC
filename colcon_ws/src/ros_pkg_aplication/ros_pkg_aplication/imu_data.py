#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import transformations as tf

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

class DataImuNode(Node):
    def __init__(self):
        super().__init__("imu_data")
        self.data_publisher1 = self.create_publisher(Vector3, 'Orientacion_IMU', 10)
        self.data_publisher2 = self.create_publisher(Vector3, 'Velocidad_angular_IMU', 10)
        self.data_publisher3 = self.create_publisher(Vector3, 'Aceleracion_lineal_IMU', 10)

        self.number_subscriber_1 = self.create_subscription(Imu, '/aiapaec/imu', self.callback_operation1, 10)

        self.get_logger().info("Data IMU has been started.")

        self.msg_orientacion = Vector3() 
        self.msg_velocidad_angular = Vector3() 
        self.msg_aceleracion_lineal = Vector3() 

        self.timer = self.create_timer(0.1, self.timer_callback) #0.1 second

    def callback_operation1(self, msg):
        new_msg_quaternion = msg.orientation
        qx, qy, qz, qw = new_msg_quaternion.x, new_msg_quaternion.y, new_msg_quaternion.z, new_msg_quaternion.w

        norm_quaternion = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        qx /= norm_quaternion
        qy /= norm_quaternion
        qz /= norm_quaternion
        qw /= norm_quaternion
        
        euler_angles = tf.euler_from_quaternion([qx, qy, qz, qw], axes='xyzs')

        self.msg_orientacion.x = np.degrees(euler_angles[0])
        self.msg_orientacion.y = np.degrees(euler_angles[1])
        self.msg_orientacion.z = np.degrees(euler_angles[2])

        self.msg_aceleracion_lineal.x = msg.linear_acceleration.x
        self.msg_aceleracion_lineal.y = msg.linear_acceleration.y
        self.msg_aceleracion_lineal.z = msg.linear_acceleration.z

        self.msg_velocidad_angular.x = msg.angular_velocity.x
        self.msg_velocidad_angular.y = msg.angular_velocity.y
        self.msg_velocidad_angular.z = -1*msg.angular_velocity.z

    def timer_callback(self):
        self.data_publisher1.publish(self.msg_orientacion) # grados sexagesimales
        self.data_publisher2.publish(self.msg_velocidad_angular) #rad/s
        self.data_publisher3.publish(self.msg_aceleracion_lineal) #m/s²

def main(args=None):
    rclpy.init(args=args)
    node = DataImuNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
