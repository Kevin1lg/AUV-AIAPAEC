from setuptools import find_packages, setup

package_name = 'ros_pkg_aplication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "probe_data = ros_pkg_aplication.probe_data:main",
            "graphics_data = ros_pkg_aplication.graphics_data:main",
            "dvl_data = ros_pkg_aplication.dvl_data:main",
            "imu_data = ros_pkg_aplication.imu_data:main",
            "odometry_data = ros_pkg_aplication.odometry_data:main",
            "zig_zag_commands = ros_pkg_aplication.zig_zag_commands:main",
            "control_odometry_data = ros_pkg_aplication.control_odometry_data:main",
            "control_sensors_data = ros_pkg_aplication.control_sensors_data:main"
        ],
    },
)
