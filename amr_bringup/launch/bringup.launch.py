"""One-shot launch for the Pi: robot_description/TF, sensors (lidar + IMU),
motor driver, lidar odometry, and EKF fusion.

    ros2 launch amr_bringup bringup.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr_bringup')

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'description.launch.py'))
    )
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sensors.launch.py'))
    )
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'ekf.launch.py'))
    )

    motor_driver = Node(
        package='amr_bringup',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'motor_params.yaml')],
    )

    return LaunchDescription([description, sensors, motor_driver, ekf])
