#!/usr/bin/env python3
"""
mpu6050_node
------------
Minimal, self-contained MPU6050 I2C driver. Publishes raw (unfiltered,
unfused) accelerometer + gyro data as sensor_msgs/Imu on /imu/data_raw,
which is exactly what config/ekf.yaml expects as its imu0 input.

No orientation is computed here (orientation_covariance[0] is set to -1,
the standard sensor_msgs/Imu convention for "orientation not provided") —
fusion into a usable yaw estimate happens in the EKF.

Gyro bias calibration: on startup, the node averages a batch of gyro
readings while the robot is assumed stationary, and subtracts that bias
from every subsequent reading. Cheap MEMS gyros like the MPU6050 almost
always have a non-zero resting bias (a few hundredths of a rad/s), which
otherwise integrates into fake yaw drift over time and can corrupt
downstream EKF/AMCL heading estimates.

IMPORTANT: keep the robot completely still for the ~2 seconds after
launch while calibration runs — moving it during that window will bake
a bad bias into the whole session.

I2C failure handling: if a read fails (e.g. Errno 121 Remote I/O error
from a loose wire or a flaky connection), this node SKIPS publishing
for that tick entirely instead of publishing fabricated zero values.
Publishing confident zeros on a failed read tells the EKF "the robot
is definitely not moving right now" even if it actually is — which is
worse than missing data, since it actively corrupts the fused estimate.
Skipping the publish lets the EKF's sensor_timeout handle the gap by
coasting on its motion model instead.

Requires: smbus2   ->  pip install smbus2   (or: sudo apt install python3-smbus)
Enable I2C on the Pi first:  sudo raspi-config -> Interface Options -> I2C
Check wiring/address:        sudo i2cdetect -y 1   (MPU6050 is normally 0x68)
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    from smbus2 import SMBus
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False

# MPU6050 registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

ACCEL_SCALE = 16384.0   # LSB/g for +-2g range (default)
GYRO_SCALE = 131.0      # LSB/(deg/s) for +-250 dps range (default)
GRAVITY = 9.80665


class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('calibration_samples', 200)

        bus_num = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value
        cal_samples = self.get_parameter('calibration_samples').value

        # Default bias (used if calibration is skipped/fails)
        self.gyro_bias = (0.0, 0.0, 0.0)

        self._bus = None
        if SMBUS_AVAILABLE:
            try:
                self._bus = SMBus(bus_num)
                # Wake the MPU6050 up — it starts in sleep mode
                self._bus.write_byte_data(self.addr, PWR_MGMT_1, 0)
                self.get_logger().info(f'MPU6050 initialised on i2c bus {bus_num}, addr {hex(self.addr)}')
                time.sleep(0.1)  # let it settle after wake-up before calibrating
                self.gyro_bias = self._calibrate_gyro(cal_samples)
            except Exception as e:
                self.get_logger().error(f'Could not open MPU6050: {e}. Publishing zeros.')
                self._bus = None
        else:
            self.get_logger().warn('smbus2 not installed — publishing zeros. '
                                    'Run: pip install smbus2 (on the Pi)')

        self._pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.create_timer(1.0 / rate, self._tick)

    @staticmethod
    def _read_word(bus, addr, reg):
        high = bus.read_byte_data(addr, reg)
        low = bus.read_byte_data(addr, reg + 1)
        val = (high << 8) | low
        if val >= 0x8000:
            val -= 0x10000
        return val

    def _calibrate_gyro(self, samples=200):
        """Average `samples` gyro readings while assumed stationary at boot,
        and store the result as a bias to subtract from every future reading."""
        self.get_logger().info(f'Calibrating gyro bias — keep the robot still ({samples} samples)...')
        sum_gx = sum_gy = sum_gz = 0.0
        count = 0
        for _ in range(samples):
            try:
                gx = self._read_word(self._bus, self.addr, GYRO_XOUT_H) / GYRO_SCALE * 0.017453293
                gy = self._read_word(self._bus, self.addr, GYRO_XOUT_H + 2) / GYRO_SCALE * 0.017453293
                gz = self._read_word(self._bus, self.addr, GYRO_XOUT_H + 4) / GYRO_SCALE * 0.017453293
                sum_gx += gx
                sum_gy += gy
                sum_gz += gz
                count += 1
            except Exception:
                pass
            time.sleep(0.01)

        if count == 0:
            self.get_logger().warn('Gyro calibration failed to read any samples — bias set to 0.')
            return 0.0, 0.0, 0.0

        bias = (sum_gx / count, sum_gy / count, sum_gz / count)
        self.get_logger().info(
            f'Gyro bias calibrated: x={bias[0]:.5f} y={bias[1]:.5f} z={bias[2]:.5f} rad/s'
        )
        return bias

    def _tick(self):
        if self._bus is None:
            # No working I2C connection at all — publish nothing rather
            # than fabricated zeros that would falsely tell the EKF the
            # robot is stationary.
            return

        try:
            ax = self._read_word(self._bus, self.addr, ACCEL_XOUT_H) / ACCEL_SCALE * GRAVITY
            ay = self._read_word(self._bus, self.addr, ACCEL_XOUT_H + 2) / ACCEL_SCALE * GRAVITY
            az = self._read_word(self._bus, self.addr, ACCEL_XOUT_H + 4) / ACCEL_SCALE * GRAVITY
            gx = self._read_word(self._bus, self.addr, GYRO_XOUT_H) / GYRO_SCALE * 0.017453293
            gy = self._read_word(self._bus, self.addr, GYRO_XOUT_H + 2) / GYRO_SCALE * 0.017453293
            gz = self._read_word(self._bus, self.addr, GYRO_XOUT_H + 4) / GYRO_SCALE * 0.017453293
            gx -= self.gyro_bias[0]
            gy -= self.gyro_bias[1]
            gz -= self.gyro_bias[2]
        except Exception as e:
            self.get_logger().warn(f'MPU6050 read failed: {e}', throttle_duration_sec=5.0)
            # Skip this tick entirely — do NOT publish fake zero data.
            # A missing update is handled gracefully by the EKF's
            # sensor_timeout; a confident false zero is not.
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # orientation not provided by this sensor
        msg.orientation_covariance[0] = -1.0

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Reasonable default covariances for a cheap MEMS IMU — tune if you
        # want the EKF to trust/distrust this sensor more or less.
        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
