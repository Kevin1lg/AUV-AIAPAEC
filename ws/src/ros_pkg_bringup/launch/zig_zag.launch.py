from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_pkg_aplication',
            executable='graphics_data',
            name='graphics_data',
            output='screen',
            parameters=[
                {'step': 0.01}, #100 Hz
                {'time mode': 'simulation'}     # Choose between two options:
                                                # 'simulation': simulation time
                                                # 'real': real time
            ],
        ),
        
        #Zig zag maneover will present an error with number 3 thruster, It just considers number 1 and 2
        #thrusters (tp1, tp2) with predeterminates value tp1:30.0 N and tp2:10.0 N
         
        TimerAction(
            period=3.0,  # Delay
            actions=[
                Node(
                    package='ros_pkg_aplication',
                    executable='zig_zag_commands',
                    name='zig_zag_commands',
                    output='screen',
                    parameters=[
                        {'angulo': 30.0}     #Curve angle in degree
                    ],
                ),
            ],
        ),
    ])