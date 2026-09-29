"""Brings up the YDLidar X2 driver and the MPU6050 IMU node. Pi-side only."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',
            'ignore_array': '',
            'baudrate': 115200,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 3,
            'abnormal_check_count': 4,
            'fixed_resolution': True,
            'reversion': True,
            'inverted': True,
            'auto_reconnect': True,
            'isSingleChannel': True,   # X2 is a single-channel model
            'intensity': False,        # X2 has no intensity data
            'support_motor_dtr': True,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 8.0,
            'range_min': 0.12,
            'frequency': 7.0,
            'invalid_range_is_inf': False,
        }],
    )

    imu_node = Node(
        package='amr_bringup',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        parameters=[{
            'i2c_bus': 1,
            'i2c_address': 0x68,
            'frame_id': 'imu_link',
            'publish_rate': 50.0,
        }],
    )

    return LaunchDescription([lidar_node, imu_node])
