#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64

class DataGraphicsNode(Node):
    def __init__(self):
        super().__init__("graphics_data")

        self.number_subscriber_1 = self.create_subscription(Vector3, 'Posicion_Odometry', self.callback_operation1, 10)
        self.number_subscriber_2 = self.create_subscription(Vector3, 'Orientacion_Odometry', self.callback_operation2, 10)
        self.number_subscriber_3 = self.create_subscription(Twist, 'Velocidad_Odometry', self.callback_operation3, 10)
        self.number_subscriber_4 = self.create_subscription(Float64, '/thruster1_joint/cmd_thrust', self.callback_operation4, 10)
        self.number_subscriber_5 = self.create_subscription(Float64, '/thruster2_joint/cmd_thrust', self.callback_operation5, 10)
        self.number_subscriber_6 = self.create_subscription(Float64, '/thruster3_joint/cmd_thrust', self.callback_operation6, 10)
        #self.number_subscriber_7 = self.create_subscription(Vector3, 'Orientacion_IMU', self.callback_operation7, 10)
        #self.number_subscriber_8 = self.create_subscription(Vector3, 'Velocidad_angular_IMU', self.callback_operation8, 10)
        #self.number_subscriber_9 = self.create_subscription(Vector3, 'Velocidad_lineal_DVL', self.callback_operation9, 10)


        self.get_logger().info("Graphics has been started.")

        self.msg_posicion = Vector3() 
        self.msg_orientacion = Vector3() 
        self.msg_velocidad = Twist()  
        self.msg_propulsores = Vector3()
        #self.msg_prueba_orientacion = Vector3()
        #self.msg_prueba_velocidad_angular_IMU = Vector3()
        #self.msg_prueba_velocidad_lineal_DVL = Vector3()    

        self.fig, self.axes = plt.subplots(2, 2, figsize=(10, 8))
        self.fig2, self.ax2 = plt.subplots(figsize=(8, 6))
        
        self.posicion_data = {'x': [], 'y': [], 'z': []}
        self.orientacion_data = {'x': [], 'y': [], 'z': []}
        self.velocidad_data = {'linear': {'x': [], 'y': [], 'z': []}, 'angular': {'x': [], 'y': [], 'z': []}}
        self.propulsores_data = {'x': [], 'y': [], 'z': []}
        self.prueba_orientacion = {'x': [], 'y': [], 'z': []}
        #self.prueba_velocidad_angular_IMU = {'x': [], 'y': [], 'z': []}
        #self.prueba_velocidad_lineal_DVL = {'x': [], 'y': [], 'z': []}

        self.time_ = 0.01 # Using to control_odometry_data 100 Hz
        #self.time_ = 0.1 # Using to control_sensors_data 10 Hz

        self.timer = self.create_timer(self.time_, self.timer_callback) 

    def callback_operation1(self, msg):
        self.msg_posicion.x = msg.x
        self.msg_posicion.y = msg.y
        self.msg_posicion.z = msg.z

    def callback_operation2(self, msg):
        self.msg_orientacion = msg

    def callback_operation3(self, msg):
        self.msg_velocidad.linear = msg.linear
        self.msg_velocidad.angular = msg.angular

    def callback_operation4(self, msg):
        if msg.data:
            self.msg_propulsores.x = msg.data
        else:
            self.get_logger().warn("Error, no se recibe data de propulsor 1.")

    def callback_operation5(self, msg):
        if msg.data:
            self.msg_propulsores.y = msg.data
        else:
            self.get_logger().warn("Error, no se recibe data de propulsor 2.")

    def callback_operation6(self, msg):
        if msg.data:
            self.msg_propulsores.z = msg.data
        else:
            self.get_logger().warn("Error, no se recibe data de propulsor 3.")

    #def callback_operation7(self, msg):
    #    self.msg_prueba_orientacion = msg
        
    #def callback_operation8(self, msg):
    #    self.msg_prueba_velocidad_angular_IMU = msg

    #def callback_operation9(self, msg):
    #    self.msg_prueba_velocidad_lineal_DVL = msg

    def timer_callback(self):

        # Data is updated to graphic

        self.posicion_data['x'].append(self.msg_posicion.x)
        self.posicion_data['y'].append(self.msg_posicion.y)
        self.posicion_data['z'].append(self.msg_posicion.z)

        self.orientacion_data['x'].append(self.msg_orientacion.x) 
        self.orientacion_data['y'].append(self.msg_orientacion.y) 
        self.orientacion_data['z'].append(self.msg_orientacion.z) 

        self.velocidad_data['linear']['x'].append(self.msg_velocidad.linear.x)
        self.velocidad_data['linear']['y'].append(self.msg_velocidad.linear.y)
        self.velocidad_data['linear']['z'].append(self.msg_velocidad.linear.z)
        self.velocidad_data['angular']['x'].append(self.msg_velocidad.angular.x) 
        self.velocidad_data['angular']['y'].append(self.msg_velocidad.angular.y)
        self.velocidad_data['angular']['z'].append(self.msg_velocidad.angular.z)

        self.propulsores_data['x'].append(self.msg_propulsores.x)
        self.propulsores_data['y'].append(self.msg_propulsores.y)
        self.propulsores_data['z'].append(self.msg_propulsores.z)

        #self.prueba_orientacion['x'].append(self.msg_prueba_orientacion.x)
        #self.prueba_orientacion['y'].append(self.msg_prueba_orientacion.y)
        #self.prueba_orientacion['z'].append(self.msg_prueba_orientacion.z)

        #self.prueba_velocidad_angular_IMU['x'].append(self.msg_prueba_velocidad_angular_IMU.x)
        #self.prueba_velocidad_angular_IMU['y'].append(self.msg_prueba_velocidad_angular_IMU.y)
        #self.prueba_velocidad_angular_IMU['z'].append(self.msg_prueba_velocidad_angular_IMU.z)

        #self.prueba_velocidad_lineal_DVL['x'].append(self.msg_prueba_velocidad_lineal_DVL.x)
        #self.prueba_velocidad_lineal_DVL['y'].append(self.msg_prueba_velocidad_lineal_DVL.y)
        #self.prueba_velocidad_lineal_DVL['z'].append(self.msg_prueba_velocidad_lineal_DVL.z)

        self.update_plots()

    def update_plots(self):

        self.axes[0, 0].clear()        
        self.axes[0, 0].plot(self.propulsores_data['x'], label='Thruster 1')
        self.axes[0, 0].plot(self.propulsores_data['y'], label='Thruster 2')
        self.axes[0, 0].plot(self.propulsores_data['z'], label='Thruster 3')
        self.axes[0, 0].set_title('Thrusters')
        #self.axes[0, 0].set_xlabel('Time(cs)') 
        self.axes[0, 0].set_ylabel('Force(N)') 
        self.axes[0, 0].legend()

        self.axes[0, 1].clear()
        self.axes[0, 1].plot(self.velocidad_data['linear']['x'], label='Surge Velocity')
        self.axes[0, 1].plot(self.velocidad_data['linear']['y'], label='Sway Velocity' )
        self.axes[0, 1].set_title('Lineal Velocity')
        #self.axes[0, 1].set_xlabel('Time(cs)') 
        self.axes[0, 1].set_ylabel('Velocity(m/s)') 
        self.axes[0, 1].legend()

        self.axes[1, 0].clear()
        #self.axes[1, 0].plot(self.orientacion_data['x'], label='Orientacion X')
        #self.axes[1, 0].plot(self.orientacion_data['y'], label='Orientacion Y')
        self.axes[1, 0].plot(self.orientacion_data['z'], label='Yaw')
        self.axes[1, 0].set_title('Orientation')
        self.axes[1, 0].set_xlabel('Time(cs)') 
        self.axes[1, 0].set_ylabel('Orientation(°)') 
        self.axes[1, 0].legend()

        self.axes[1, 1].clear()
        self.axes[1, 1].plot(self.velocidad_data['angular']['x'], label='Angular velocity roll')
        self.axes[1, 1].plot(self.velocidad_data['angular']['y'], label='Angular velocity pitch')
        self.axes[1, 1].plot(self.velocidad_data['angular']['z'], label='Angular velocity yaw')
        self.axes[1, 1].set_title('Angular Velocity')
        self.axes[1, 1].set_xlabel('Time(cs)') 
        self.axes[1, 1].set_ylabel('Velocity(rad/s)') 
        self.axes[1, 1].legend()

        self.ax2.clear()
        self.ax2.plot(self.posicion_data['y'], self.posicion_data['x'], label='y vs x')
        self.ax2.set_title('Position')
        self.ax2.set_xlabel('Y(m)')
        self.ax2.set_ylabel('X(m)')
        self.ax2.legend()

        self.fig2.canvas.draw()
        self.fig2.canvas.flush_events()

        plt.pause(self.time_) #Update

def main(args=None):
    rclpy.init(args=args)
    node = DataGraphicsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
