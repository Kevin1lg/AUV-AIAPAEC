from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import numpy as np

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
        TimerAction(
            period=4.0,  # Delay
            actions=[
                Node(
                    package='ros_pkg_aplication',
                    executable='test_data',
                    name='test_data',
                    output='screen',
                    parameters=[
                        {'tp1': [30.0]},# right thruster TP1
                        {'tp2': [30.0]}, # left thruster TP2 
                        {'tp3': [0.0]}, # center thruster TP3
                        {'t': 0.01}
                    ]
                ),
            ],
        ),
    ])