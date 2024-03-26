#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3

class DataControlSensorNode(Node):
    def __init__(self):
        super().__init__("control_sensors_data") #Node name

        self.subscription = self.create_subscription(
            Vector3, '/Velocidad_lineal_DVL', self.callback_operation1, 10)
        self.subscription = self.create_subscription(
            Vector3, '/Velocidad_angular_IMU', self.callback_operation2, 10)

        self.publisher1_ = self.create_publisher(
            Float64, "/thruster1_joint/cmd_thrust", 10)  
        self.publisher2_ = self.create_publisher(
            Float64, "/thruster2_joint/cmd_thrust", 10)  
        self.publisher3_ = self.create_publisher(
            Float64, "/thruster3_joint/cmd_thrust", 10)

        self.get_logger().info("Control function with sensors data has been started.")

        self.msg_velocidad = Twist()
        self.counter_0 = 0

        self.timer_ = self.create_timer(0.1, self.publish_data)  # 0.1 seconds

    def callback_operation1(self, msg):
        self.msg_velocidad.linear.x = msg.x 
        self.msg_velocidad.linear.y = msg.y

    def callback_operation2(self, msg):
        self.msg_velocidad.angular.z = msg.z

    def Control(self, u, v, r):  # Control function with 0.1 second sampling time

        global X

        ur = 1.0  # 1 m/s
        vr = 0.0  # 0 m/s
        rr = 0.3  # 0.3 rad/s

        AK=np.array([
            [0.607337878841786,     0.0746448081858653,     0.0647638448417243,     -4.30152148794906,  -2.19811298674322,      6.93729557764431,       0, 0, 0],
            [0.0582305777643910,    0.676591288694805,      -0.0573600589610847,    -16.4591247818208,  0.835897817211827,      -5.32680534464987,      0, 0, 0],
            [0.0450351417303052,    -0.0369987101013010,    0.374714821117937,      2.06304453905126,   -7.19679312512331,      -3.07184630431515,      0, 0, 0],
            [0.000813086328118496,  0.000873898175783859,   1.15946720822427e-05,   0.349351500522081,  -0.000908907991582862, 0.00205508061670335,     0, 0, 0],
            [-2.72706115947119e-05, 6.60373677279223e-06,   0.00505633889825682,    -0.0302773989877139, 0.287784862469284,     -0.0217041819251470,    0, 0, 0],
            [-0.00206657181856424,  0.00198581460210438,    0.00419669738778251,    -0.132636920993801, -0.100017334202740,     0.329551599762116,      0, 0, 0],
            [-0.405426227646965,    0.0629290717652940,     0.0278548966721269,     -18.5572786903498,  -8.72602819273259,      7.72332800333625,       1, 0, 0],
            [0.0759667496660267,    -0.310338422455651,     -0.0257405436484186,    1.61971349334845,   7.28066520247026,       -6.93384471822468,      0, 1, 0],
            [0.0483166544997876,    -0.0341646535018950,    -0.606568133234333,     5.58462449190233,   -3.92806893517125,      -3.42439107058872,      0, 0, 1]
        ])

        BK=np.array([
            [9.60991500746873,      92.3559510018235,   -31.1572347735694],
            [-6.32089808284193,     -87.1365344720799,  31.7095494331322],
            [-1.82667936349399,     22.0020832597618,   14.7621763912017],
            [-0.313638370901265,    0.00178085383245549,-0.00208418775423546],
            [5.10616083021119e-05,  -6.09512548772342,  0.0623047175568620],
            [0.000617425924256624,  0.149335867624582,  -3.50300136134665],
            [2.28089850058081,      22.1540034264146,   -27.1107444480999],
            [2.88376049825568,      -18.7876945745824,  22.7434812335434],
            [0.00953511199237775,   57.1875052038549,   13.0211215831943]
        ])

        CK=np.array([
            [0,	0,	0,	0,	0,	0,	1,	0,	0],
            [0,	0,	0,	0,	0,	0,	0,	1,	0],
            [0,	0,	0,	0,	0,	0,	0,	0,	1]
        ])

        DK=np.array([
            [0,	0,	0],
            [0,	0,	0],
            [0,	0,	0]
        ])

        VV = np.array([[u],
                       [v],
                       [r]])
        
        VVR = np.array([[ur],
                        [vr],
                        [rr]])

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
    node = DataControlSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
