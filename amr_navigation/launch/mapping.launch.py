"""Run this FIRST to build a map by driving the robot around.

    ros2 launch amr_navigation mapping.launch.py

Runs slam_toolbox (online async) + RViz. Assumes amr_bringup's
bringup.launch.py is already running on the Pi, publishing /scan,
/odometry/filtered and TF.

Drive it around with, e.g.:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

When you're happy with the map:
    ros2 run nav2_map_server map_saver_cli -f ~/ros_ws/src/amr_navigation/maps/my_map
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('amr_navigation')
    desc_pkg = get_package_share_directory('amr_description')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(nav_pkg, 'config', 'slam_toolbox_params.yaml')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'amr.rviz')],
    )

    return LaunchDescription([slam, rviz])
