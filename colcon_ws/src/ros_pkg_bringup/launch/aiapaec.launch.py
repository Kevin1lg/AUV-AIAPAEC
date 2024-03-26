import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_pkg_bringup')
    pkg_project_description = get_package_share_directory('aiapaec_models')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'aiapaec', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world // Corregir tópicos
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_description,
            'worlds',
            'aiapaec_underwater.world'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )
    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    odometry_data = Node(
        package='ros_pkg_aplication',
        executable='odometry_data',
        name='odometry_data',
        output='screen'
    )

    imu_data = Node(
        package='ros_pkg_aplication',
        executable='imu_data',
        name='imu_data',
        output='screen'
    )
    publisher = ExecuteProcess(
        cmd=['./build/gz-transport-dvl/publisher'],
        output='screen',
    )
    dvl_data = Node(
        package='ros_pkg_aplication',
        executable='dvl_data',
        name='dvl_data',
        output='screen'
    )

    graphics_data = Node(
        package='ros_pkg_aplication',
        executable='graphics_data',
        name='graphics_data',
        output='screen'
    )
    
    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        robot_state_publisher,
        odometry_data,
        imu_data,
        publisher,
        dvl_data,
        graphics_data
    ])
