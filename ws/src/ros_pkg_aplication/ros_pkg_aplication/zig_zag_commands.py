#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

class DataZigZagNode(Node):
    def __init__(self):
        super().__init__("zig_zag_commands")

        self.number_subscriber_1 = self.create_subscription(Vector3, '/Posicion_Odometry', self.callback_operation1, 10)
        self.number_subscriber_2 = self.create_subscription(Vector3, '/Orientacion_Odometry', self.callback_operation2, 10)

        self.number_publisher1_ = self.create_publisher(Float64,"/thruster1_joint/cmd_thrust",10) 
        self.number_publisher2_ = self.create_publisher(Float64,"/thruster2_joint/cmd_thrust",10) 
        self.number_publisher3_ = self.create_publisher(Float64,"/thruster3_joint/cmd_thrust",10)

        # Declare the parameter
        self.declare_parameter('angulo', 0.0)

        # Get the parameter
        self.angulo = self.get_parameter('angulo').get_parameter_value()
 
        self.number_timer_ = self.create_timer(0.01, self.publish_data) # 100 Hz , 0.01 sec the step
        self.get_logger().info('Zig Zag maneuver has been started with angulo {}°'.format(self.angulo.double_value))

        self.msg_orientacion = Vector3()
        self.msg_posicion = Vector3()

    def callback_operation1(self,msg):
        self.msg_posicion = msg

    def callback_operation2(self,msg):        
        self.msg_orientacion = msg 

    def Zig_Zag(self, yaw, pos_y): # Zig Zag maneuver function
        if yaw >= self.angulo.double_value:
            TP = np.array([30.0, 10.0, 0.0])  # Curve to the left
        elif yaw <= -self.angulo.double_value:
            TP = np.array([10.0, 30.0, 0.0])  # Curve to the right
        else:
            if pos_y % 20 < 10:
                TP = np.array([10.0, 30.0, 0.0])  # Curve to the right
            else:
                TP = np.array([30.0, 10.0, 0.0])  # Curve to the left
        return TP

    def publish_data(self):
        msg1 = Float64()
        msg2 = Float64()
        msg3 = Float64()

        TP = self.Zig_Zag(self.msg_orientacion.z,self.msg_posicion.y)

        TP1, TP2, TP3 = TP[0], TP[1], TP[2]

        ## Thrusters Forces (Newton)
        
        msg1.data = TP1
        msg2.data = TP2
        msg3.data = TP3

        self.number_publisher1_.publish(msg1)
        self.number_publisher2_.publish(msg2)
        self.number_publisher3_.publish(msg3)

def main(args=None):
    rclpy.init(args=args)
    node = DataZigZagNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()