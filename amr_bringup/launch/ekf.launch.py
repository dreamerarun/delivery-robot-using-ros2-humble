"""Lidar-scan-matching odometry (rf2o) + EKF fusion with the IMU. No wheel
encoders on this build, so rf2o stands in for wheel odometry."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr_bringup')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,          # EKF publishes odom->base_link, not rf2o
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    return LaunchDescription([rf2o, ekf])
