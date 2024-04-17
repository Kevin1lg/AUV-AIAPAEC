#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import transformations as tf

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

class DataOdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_data")
        self.data_publisher1 = self.create_publisher(Vector3,'Posicion_Odometry',10)
        self.data_publisher2 = self.create_publisher(Vector3,'Orientacion_Odometry',10)
        self.data_publisher3 = self.create_publisher(Twist,'Velocidad_Odometry',10)

        self.number_subscriber_1 = self.create_subscription(Odometry,'/aiapaec/odometry',self.callback_operation1, 10)

        self.get_logger().info("Data Odometry has been started.")

        self.msg_posicion = Vector3() 
        self.msg_orientacion = Vector3() 
        self.msg_velocidad = Twist() 
    
        self.timer = self.create_timer(0.01, self.timer_callback) # 0.01 second
    
    def callback_operation1(self, msg):
        
        # Posicion
        self.msg_posicion.x = msg.pose.pose.position.x
        self.msg_posicion.y = -1*msg.pose.pose.position.y
        self.msg_posicion.z = msg.pose.pose.position.z

        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w

        norm_quaternion = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        qx /= norm_quaternion
        qy /= norm_quaternion
        qz /= norm_quaternion
        qw /= norm_quaternion
        
        euler_angles = tf.euler_from_quaternion([qx, qy, qz, qw], axes='xyzs')

        self.msg_orientacion.x = np.degrees(euler_angles[0]) # roll
        self.msg_orientacion.y = np.degrees(euler_angles[1]) # pitch
        self.msg_orientacion.z = np.degrees(euler_angles[2]) # yaw

        #Velocidad angular

        self.msg_velocidad.angular.x = msg.twist.twist.angular.x
        self.msg_velocidad.angular.y = msg.twist.twist.angular.y
        self.msg_velocidad.angular.z = -1*msg.twist.twist.angular.z

        #Velocidad lineal

        self.msg_velocidad.linear = msg.twist.twist.linear


    def timer_callback(self):
        self.data_publisher1.publish(self.msg_posicion)
        self.data_publisher2.publish(self.msg_orientacion)
        self.data_publisher3.publish(self.msg_velocidad)


def main(args=None):
    rclpy.init(args=args)
    node = DataOdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
