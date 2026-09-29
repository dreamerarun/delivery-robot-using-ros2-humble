"""Publishes robot_description + static TF (base_link -> laser_frame,
base_link -> imu_link, etc). Runs on the Pi — this is the single source
of truth for TF; the laptop's RViz subscribes to it remotely, it does not
need its own copy running."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    pkg = get_package_share_directory('amr_bringup')
    xacro_file = os.path.join(pkg, 'urdf', 'amr.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    return LaunchDescription([rsp])
