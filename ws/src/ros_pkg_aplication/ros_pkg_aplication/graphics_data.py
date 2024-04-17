#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64
from builtin_interfaces.msg import Time

class DataGraphicsNode(Node):
    def __init__(self):
        super().__init__("graphics_data")

        # Declare the parameter
        self.declare_parameter('step', 0.0)
        self.declare_parameter('time mode', 'simulation') # simulation or real

        # Get the parameter
        period = self.get_parameter('step').get_parameter_value()
        mode_ = self.get_parameter('time mode').get_parameter_value()

        self.number_subscriber_1 = self.create_subscription(Vector3, 'Posicion_Odometry', self.callback_operation1, 10)
        self.number_subscriber_2 = self.create_subscription(Vector3, 'Orientacion_Odometry', self.callback_operation2, 10)
        self.number_subscriber_3 = self.create_subscription(Twist, 'Velocidad_Odometry', self.callback_operation3, 10)
        self.number_subscriber_4 = self.create_subscription(Float64, '/thruster1_joint/cmd_thrust', self.callback_operation4, 10)
        self.number_subscriber_5 = self.create_subscription(Float64, '/thruster2_joint/cmd_thrust', self.callback_operation5, 10)
        self.number_subscriber_6 = self.create_subscription(Float64, '/thruster3_joint/cmd_thrust', self.callback_operation6, 10)
        #self.number_subscriber_7 = self.create_subscription(Vector3, 'Orientacion_IMU', self.callback_operation7, 10)
        #self.number_subscriber_8 = self.create_subscription(Vector3, 'Velocidad_angular_IMU', self.callback_operation8, 10)
        #self.number_subscriber_9 = self.create_subscription(Vector3, 'Velocidad_lineal_DVL', self.callback_operation9, 10)
        self.number_subscriber_10 = self.create_subscription(Time, '/aiapaec/real_time', self.callback_operation10, 10)
        self.number_subscriber_11 = self.create_subscription(Time, '/aiapaec/sim_time', self.callback_operation11, 10)
        self.number_subscriber_12 = self.create_subscription(Float64, '/aiapaec/rtf', self.callback_operation12, 10)

        self.get_logger().info("Graphics has been started.")

        self.msg_posicion = Vector3() 
        self.msg_orientacion = Vector3() 
        self.msg_velocidad = Twist()  
        self.msg_propulsores = Vector3()
        #self.msg_prueba_orientacion = Vector3()
        #self.msg_prueba_velocidad_angular_IMU = Vector3()
        #self.msg_prueba_velocidad_lineal_DVL = Vector3()
        self.msg_real_time = Time()
        self.msg_sim_time = Time()
        self.msg_rtf = Float64()  

        self.initial_mode_sim = None  
        self.initial_mode_real = None  

        self.fig, self.axes = plt.subplots(1, 2, figsize=(12, 8)) #Position and Orientation
        self.fig1, self.axes1 = plt.subplots(figsize=(14, 10)) # Lineal Velocity
        self.fig2, self.axes2 = plt.subplots(figsize=(14, 10)) # Angular Velocity
        self.fig3, self.axes3 = plt.subplots(figsize=(14, 10)) # Thrusters
        # self.fig4, self.axes4 = plt.subplots(figsize=(14, 10)) # Real Time Factor
        
        self.posicion_data = {'x': [], 'y': [], 'z': []}
        self.orientacion_data = {'roll': [], 'pitch': [], 'yaw': []}
        self.velocidad_data = {'linear': {'x': [], 'y': [], 'z': []}, 'angular': {'x': [], 'y': [], 'z': []}}
        self.propulsores_data = {'tp1': [], 'tp2': [], 'tp3': []}
        #self.prueba_orientacion = {'roll_p': [], 'pitch_p': [], 'yaw_p': []}
        #self.prueba_velocidad_angular_IMU = {'x': [], 'y': [], 'z': []}
        #self.prueba_velocidad_lineal_DVL = {'x': [], 'y': [], 'z': []}
        self.real_time = {'sec':[], 'nsec':[], 'total':[]} #total = sec + nsec
        self.sim_time = {'sec':[], 'nsec':[], 'total':[]} #total = sec + nsec
        self.rtf = {'rtf':[]} #total = sec + nsec

        self.time_ = period.double_value # Sampling Time

        # Define if the time is real or simulation according gazebo enviroment
        if mode_.string_value == 'simulation':
            self.mode = self.sim_time
            self.get_logger().info("Mode: Simulation Time")
        if mode_.string_value == 'real':
            self.mode = self.real_time
            self.get_logger().info("Mode: Real Time")

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

    def callback_operation10(self, msg):
        self.msg_real_time.nanosec = msg.nanosec
        self.msg_real_time.sec = msg.sec

        if self.initial_mode_real is None:
            self.initial_mode_real = self.msg_real_time.nanosec/1e9 + self.msg_real_time.sec
    
    def callback_operation11(self, msg):
        self.msg_sim_time.nanosec = msg.nanosec
        self.msg_sim_time.sec = msg.sec

        if self.initial_mode_sim is None:
            self.initial_mode_sim = self.msg_sim_time.nanosec/1e9 + self.msg_sim_time.sec
    
    def callback_operation12(self, msg):
        self.msg_rtf = msg

    def timer_callback(self):

        # Data is updated to graphic

        self.posicion_data['x'].append(self.msg_posicion.x)
        self.posicion_data['y'].append(self.msg_posicion.y)
        self.posicion_data['z'].append(self.msg_posicion.z)

        self.orientacion_data['roll'].append(self.msg_orientacion.x) 
        self.orientacion_data['pitch'].append(self.msg_orientacion.y) 
        self.orientacion_data['yaw'].append(self.msg_orientacion.z) 

        self.velocidad_data['linear']['x'].append(self.msg_velocidad.linear.x)
        self.velocidad_data['linear']['y'].append(self.msg_velocidad.linear.y)
        self.velocidad_data['linear']['z'].append(self.msg_velocidad.linear.z)
        self.velocidad_data['angular']['x'].append(self.msg_velocidad.angular.x) 
        self.velocidad_data['angular']['y'].append(self.msg_velocidad.angular.y)
        self.velocidad_data['angular']['z'].append(self.msg_velocidad.angular.z)

        self.propulsores_data['tp1'].append(self.msg_propulsores.x)
        self.propulsores_data['tp2'].append(self.msg_propulsores.y)
        self.propulsores_data['tp3'].append(self.msg_propulsores.z)

        #self.prueba_orientacion['roll_p'].append(self.msg_prueba_orientacion.x)
        #self.prueba_orientacion['pitch_p'].append(self.msg_prueba_orientacion.y)
        #self.prueba_orientacion['yaw_p'].append(self.msg_prueba_orientacion.z)

        #self.prueba_velocidad_angular_IMU['x'].append(self.msg_prueba_velocidad_angular_IMU.x)
        #self.prueba_velocidad_angular_IMU['y'].append(self.msg_prueba_velocidad_angular_IMU.y)
        #self.prueba_velocidad_angular_IMU['z'].append(self.msg_prueba_velocidad_angular_IMU.z)

        #self.prueba_velocidad_lineal_DVL['x'].append(self.msg_prueba_velocidad_lineal_DVL.x)
        #self.prueba_velocidad_lineal_DVL['y'].append(self.msg_prueba_velocidad_lineal_DVL.y)
        #self.prueba_velocidad_lineal_DVL['z'].append(self.msg_prueba_velocidad_lineal_DVL.z)

        self.sim_time['sec'].append(self.msg_sim_time.sec)
        self.sim_time['nsec'].append(self.msg_sim_time.nanosec)
        # seconds + nanoseconds - offset to the user = total
        self.sim_time['total'].append(self.msg_sim_time.nanosec/1e9 + self.msg_sim_time.sec - self.initial_mode_sim)


        self.real_time['sec'].append(self.msg_real_time.sec)
        self.real_time['nsec'].append(self.msg_real_time.nanosec)
        # seconds + nanoseconds - offset to the user = total
        self.real_time['total'].append(self.msg_real_time.nanosec/1e9 + self.msg_real_time.sec - self.initial_mode_real)
      
        self.rtf['rtf'].append(self.msg_rtf.data)

        self.update_plots()

    def update_plots(self):

        self.axes[0].clear()        
        self.axes[0].plot(self.posicion_data['y'], self.posicion_data['x'], label='y vs x')
        self.axes[0].set_title('Position')
        self.axes[0].set_xlabel('X(m)') 
        self.axes[0].set_ylabel('Y(m)') 
        self.axes[0].legend()
        self.axes[0].grid()

        self.axes[1].clear()
        #self.axes[1].plot(self.orientacion_data['roll'], label='Roll')
        #self.axes[1].plot(self.orientacion_data['pitch'], label='Pitch')
        self.axes[1].plot(self.mode['total'], self.orientacion_data['yaw'], label='Yaw')
        self.axes[1].set_title('Orientation')
        self.axes[1].set_xlabel('Time(s)') 
        self.axes[1].set_ylabel('Orientation(°)') 
        self.axes[1].legend()
        self.axes[1].grid()

        self.axes1.clear()
        self.axes1.plot(self.mode['total'], self.velocidad_data['linear']['x'], label='Surge Velocity')
        self.axes1.plot(self.mode['total'], self.velocidad_data['linear']['y'], label='Sway Velocity' )
        self.axes1.plot(self.mode['total'], self.velocidad_data['linear']['z'], label='Heavy Velocity' )
        self.axes1.set_title('Lineal Velocity')
        self.axes1.set_xlabel('Time(s)')  
        self.axes1.set_ylabel('Velocity(m/s)') 
        self.axes1.legend()
        self.axes1.grid()


        self.fig1.canvas.draw()
        self.fig1.canvas.flush_events()

        self.axes2.clear()
        self.axes2.plot(self.mode['total'], self.velocidad_data['angular']['x'], label='Angular velocity roll')
        self.axes2.plot(self.mode['total'], self.velocidad_data['angular']['y'], label='Angular velocity pitch')
        self.axes2.plot(self.mode['total'], self.velocidad_data['angular']['z'], label='Angular velocity yaw')
        self.axes2.set_title('Angular Velocity')
        self.axes2.set_xlabel('Time(s)')  
        self.axes2.set_ylabel('Velocity(rad/s)') 
        self.axes2.legend()
        self.axes2.grid()

        self.fig2.canvas.draw()
        self.fig2.canvas.flush_events()

        self.axes3.clear()
        self.axes3.plot(self.mode['total'], self.propulsores_data['tp1'], label='Thruster 1')
        self.axes3.plot(self.mode['total'], self.propulsores_data['tp2'], label='Thruster 2')
        self.axes3.plot(self.mode['total'], self.propulsores_data['tp3'], label='Thruster 3')
        self.axes3.set_title('Thrusters')
        self.axes3.set_xlabel('Time(s)')  
        self.axes3.set_ylabel('Force(N)') 
        self.axes3.legend()
        self.axes3.grid()

        self.fig3.canvas.draw()
        self.fig3.canvas.flush_events()

        # The RTF need to revision, be careful with your values

        #self.axes4.clear()
        #self.axes4.plot(self.mode['total'],self.rtf['rtf'])
        #self.axes4.set_title('RTF')
        #self.axes4.set_xlabel('Time(s)')  
        #self.axes4.grid()

        #self.fig4.canvas.draw()
        #self.fig4.canvas.flush_events()

        plt.pause(self.time_) #Update

def main(args=None):
    rclpy.init(args=args)
    node = DataGraphicsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
