#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class DataControlNode(Node):
    def __init__(self):
        super().__init__("control_odometry_data")

        self.subscription = self.create_subscription(
            Twist, '/Velocidad_Odometry', self.callback_operation1, 10)

        self.publisher1_ = self.create_publisher(
            Float64, "/thruster1_joint/cmd_thrust", 10) 
        self.publisher2_ = self.create_publisher(
            Float64, "/thruster2_joint/cmd_thrust", 10) 
        self.publisher3_ = self.create_publisher(
            Float64, "/thruster3_joint/cmd_thrust", 10)

        self.get_logger().info("Control function with odometry data has been started.")

        self.msg_velocidad = Twist()
        self.counter_0 = 0

        # Declare the parameter
        self.declare_parameter('ur', 0.0)
        self.declare_parameter('vr', 0.0)
        self.declare_parameter('rr', 0.0)

        # Get the parameter
        self.ur_ = self.get_parameter('ur').get_parameter_value()
        self.vr_ = self.get_parameter('vr').get_parameter_value()
        self.rr_ = self.get_parameter('rr').get_parameter_value()

        self.timer_ = self.create_timer(0.01, self.publish_data)  # 0.01 second

    def callback_operation1(self, msg):
        self.msg_velocidad = msg  

    def Control(self, u, v, r):  # Control function with 0.01 second sampling time

        global X

        #ur = 1.0  # 1 m/s
        #vr = 0.0  # 0 m/s
        #rr = 0.3  # 0.3 rad/s // 0.00349

        AK = np.array([
            [0.953532860783369, 0.00916903256383125, 0.00696834747802615, -0.502019354275720, -0.249063310686180, 1.43575016190742, 0, 0, 0],
            [0.00886718138798180, 0.964777806140999, -0.00564384521364780, -3.12861013531519, 0.0222104886149861, -1.06350422361683, 0, 0, 0],
            [0.00649544788582733, -0.00521987414512458, 0.921208713590625, 0.203258634912841, -1.83689010598935, -0.823870580333387, 0, 0, 0],
            [0.000151486766497796, 0.000152499535312482, 2.57698891681842e-07, 0.903672806340970, 2.12089327623440e-05, 0.000227577214733927, 0, 0, 0],
            [-4.62423010317721e-05, 4.57783280015512e-05, 0.00125763289981952, -0.00970093548722452, 0.895649972221563, 0.000169129281556941, 0, 0, 0],
            [-0.000440130686030489, 0.000438676751356104, 0.00113784291985273, -0.0295999686333330, -0.0167033092391212, 0.905875640234624, 0, 0, 0],
            [-0.0466459379081346, 0.00898834079220941, 0.00632226026939166, -2.69893865540218, -1.29167793767722, 1.54554585172417, 1, 0, 0],
            [0.00912296623205487, -0.0350263750195422, -0.00509505163280320, -0.366228901178719, 1.03932515209987, -1.30115572276478, 0, 1, 0],
            [0.00653833766212277, -0.00517273061384376, -0.0784629561608256, 0.752608544815742, -1.31440976464264, -0.871581205415559, 0, 0, 1]
        ])

        BK = np.array([
            [1.13478909427294, 10.7971926778475, -1.06261766593043],
            [-1.34129054487973, -10.4718326016046, 1.71637274276722],
            [-0.275741053675355, -4.28001681160397, 0.514704043872406],
            [-0.0475772718511337, 3.10814216125600e-06, -4.20377712076673e-06],
            [1.69416346346585e-07, -0.951196913924195, 0.000123677113324406],
            [1.21129439259679e-06, 0.000304535535415495, -0.545018339537521],
            [0.0338038907084646, 0.331184656549127, -0.431300381773722],
            [0.0414954140015434, -0.273557187092841, 0.352701455544982],
            [9.86420888655005e-05, 0.965505316251752, 0.240564407806621]
        ])

        CK = np.array([
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1]
        ])

        DK = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ])

        VV = np.array([[u],
                       [v],
                       [r]])
        
        VVR = np.array([[self.ur_.double_value],
                        [self.vr_.double_value],
                        [self.rr_.double_value]])


        E = VVR - VV

        if self.counter_0 == 0:
            X = np.array([[0.001], 
                          [0.001], 
                          [0.001], 
                          [0.001], 
                          [0.001],
                          [0.001], 
                          [0.001], 
                          [0.001], 
                          [0.001]])

        U = np.dot(CK, X) + np.dot(DK, E)
        X = np.dot(AK, X) + np.dot(BK, E)

        self.counter_0 += 1

        return U

    def publish_data(self):

        msg1 = Float64()
        msg2 = Float64()
        msg3 = Float64()

        TP = self.Control(self.msg_velocidad.linear.x,
                           self.msg_velocidad.linear.y, 
                           self.msg_velocidad.angular.z)

        TP1, TP2, TP3 = np.float64(TP[0, 0]), np.float64(TP[1, 0]), np.float64(TP[2, 0])

        msg1.data = TP1
        msg2.data = TP2
        msg3.data = TP3

        self.publisher1_.publish(msg1)
        self.publisher2_.publish(msg2)
        self.publisher3_.publish(msg3)

def main(args=None):
    rclpy.init(args=args)
    node = DataControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
