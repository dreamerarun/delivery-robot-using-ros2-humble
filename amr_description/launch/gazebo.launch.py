"""Optional: full Gazebo (Harmonic) sim of the AMR, laptop-only, for testing
nav logic before/without the real robot. Not required for the real robot to
work — that's amr_bringup + amr_navigation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr_description')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    xacro_file = os.path.join(pkg, 'urdf', 'amr.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'amr_world.sdf')
    robot_description = Command(['xacro ', xacro_file])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description}])

    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'amr', '-z', '0.1'])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': os.path.join(pkg, 'config', 'amr_bridge.yaml')}])

    return LaunchDescription([gz_sim, rsp, spawn, bridge])
