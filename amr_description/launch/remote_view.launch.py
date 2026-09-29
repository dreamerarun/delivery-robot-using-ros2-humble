"""RViz view of the real robot running on the Pi. No local robot_state_publisher
is started here — the Pi's amr_bringup already publishes robot_description
and TF, and RViz subscribes to those remotely over DDS.

Requirements: laptop and Pi on the same network/subnet, same ROS_DOMAIN_ID
(and same ROS_LOCALHOST_ONLY=0, or unset, on both machines).

    ros2 launch amr_description remote_view.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr_description')
    rviz_config = os.path.join(pkg, 'rviz', 'amr.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([rviz])
