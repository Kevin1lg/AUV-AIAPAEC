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

        TimerAction(
            period=4.0,  # Delay
            actions=[
                Node(
                    package='ros_pkg_aplication',
                    executable='control_odometry_data',
                    name='control_odometry_data',
                    output='screen',
                    parameters=[
                        {'ur': 1.0},  #1.0 m/s
                        {'vr': 0.175},#0.175 m/s
                        {'rr': 0.0},  #0.0 rad/s
                    ]
                ),
            ],
        ),
    ])