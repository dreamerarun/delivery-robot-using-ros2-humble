"""Local sanity check of the URDF — no robot, no Pi needed, just RViz +
joint_state_publisher_gui so you can eyeball the model and wiggle the wheels."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    pkg = get_package_share_directory('amr_description')
    xacro_file = os.path.join(pkg, 'urdf', 'amr.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])
    rviz_config = os.path.join(pkg, 'rviz', 'amr.rviz')

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
    ])
