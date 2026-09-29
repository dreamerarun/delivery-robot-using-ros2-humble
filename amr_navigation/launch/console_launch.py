"""Run this on the laptop instead of rosbridge_websocket_launch.xml directly.
Starts rosbridge AND the launch-manager node together, so the console's
Start mapping / Start navigation / Stop buttons have something to talk to.

    ros2 launch amr_navigation console_launch.py

Only other thing you should need to run by hand is bringup on the Pi:

    ros2 launch amr_bringup bringup.launch.py

Do not separately run ekf.launch.py — it's already included by bringup.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
    )
    manager = Node(
        package='amr_navigation',
        executable='launch_manager_node',
        name='launch_manager_node',
        output='screen',
    )
    return LaunchDescription([rosbridge, manager])
