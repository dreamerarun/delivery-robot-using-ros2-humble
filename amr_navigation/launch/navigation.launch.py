"""Run this AFTER you have a saved map, for autonomous navigation.

    ros2 launch amr_navigation navigation.launch.py map:=/path/to/my_map.yaml

Brings up the full Nav2 stack (map_server, amcl, planner, controller,
behavior_server, bt_navigator, velocity_smoother, lifecycle manager) plus
RViz with the Nav2 "2D Goal Pose" tool. Assumes amr_bringup's
bringup.launch.py is already running on the Pi.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('amr_navigation')
    desc_pkg = get_package_share_directory('amr_description')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(nav_pkg, 'maps', 'my_map.yaml')

    map_arg = DeclareLaunchArgument('map', default_value=default_map,
                                     description='Full path to the saved map yaml')
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_pkg, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 params file')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'amr.rviz')],
    )

    return LaunchDescription([map_arg, params_arg, nav2, rviz])
