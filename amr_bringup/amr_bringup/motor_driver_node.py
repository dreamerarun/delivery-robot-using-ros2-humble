#!/usr/bin/env python3
"""
motor_driver_node
------------------
Subscribes to /cmd_vel (geometry_msgs/Twist) and drives two BO motors
through an L298N H-bridge on the Raspberry Pi 3B using RPi.GPIO
software PWM.

There are no wheel encoders in this build, so this node is open-loop:
it converts (linear.x, angular.z) into left/right wheel speeds via the
standard differential-drive kinematics, maps those speeds to a PWM
duty cycle, and sets direction pins accordingly. Actual odometry for
SLAM/Nav2 comes from rf2o_laser_odometry + the IMU, fused in
robot_localization (see ekf.launch.py) — NOT from this node.

Wire-up (BCM numbering, change in config/motor_params.yaml if yours differs):

    L298N            Raspberry Pi 3B (BCM)
    ---------------------------------------
    ENA (left PWM)    GPIO 12
    IN1 (left dir)     GPIO 5
    IN2 (left dir)     GPIO 6
    ENB (right PWM)   GPIO 13
    IN3 (right dir)    GPIO 19
    IN4 (right dir)    GPIO 26
    GND                Pi GND  (common ground with motor battery!)

Safety: publishing stops (or the topic going silent) triggers an
automatic stop via a watchdog timer, so a dropped connection doesn't
leave the motors running.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    # Lets you sanity-check the node's logic on a laptop without RPi.GPIO
    # installed. On the actual Pi this will always succeed.
    GPIO_AVAILABLE = False


class DummyGPIO:
    """No-op stand-in for RPi.GPIO when not running on a Pi."""
    BCM = OUT = LOW = HIGH = None

    def setmode(self, *a, **k): pass
    def setwarnings(self, *a, **k): pass
    def setup(self, *a, **k): pass
    def output(self, *a, **k): pass
    def cleanup(self, *a, **k): pass

    class PWM:
        def __init__(self, *a, **k): pass
        def start(self, *a, **k): pass
        def ChangeDutyCycle(self, *a, **k): pass
        def stop(self, *a, **k): pass


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # ---- parameters ----------------------------------------------
        self.declare_parameter('wheel_separation', 0.18)   # metres
        self.declare_parameter('wheel_radius', 0.0325)      # metres (BO motor wheel)
        self.declare_parameter('max_wheel_speed', 0.35)     # m/s, tune to your motors
        self.declare_parameter('min_duty_cycle', 35.0)      # % duty needed to overcome friction
        self.declare_parameter('cmd_timeout', 0.5)          # seconds, watchdog

        self.declare_parameter('left_pwm_pin', 12)
        self.declare_parameter('left_in1_pin', 5)
        self.declare_parameter('left_in2_pin', 6)
        self.declare_parameter('right_pwm_pin', 13)
        self.declare_parameter('right_in3_pin', 19)
        self.declare_parameter('right_in4_pin', 26)
        self.declare_parameter('pwm_frequency', 1000)       # Hz

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.min_duty = self.get_parameter('min_duty_cycle').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        left_pwm_pin = self.get_parameter('left_pwm_pin').value
        left_in1 = self.get_parameter('left_in1_pin').value
        left_in2 = self.get_parameter('left_in2_pin').value
        right_pwm_pin = self.get_parameter('right_pwm_pin').value
        right_in3 = self.get_parameter('right_in3_pin').value
        right_in4 = self.get_parameter('right_in4_pin').value
        pwm_freq = self.get_parameter('pwm_frequency').value

        self._gpio = GPIO if GPIO_AVAILABLE else DummyGPIO()
        if not GPIO_AVAILABLE:
            self.get_logger().warn(
                'RPi.GPIO not available — running in simulation/log-only mode. '
                'This is expected on a laptop, NOT on the Pi.')

        self._gpio.setmode(self._gpio.BCM)
        self._gpio.setwarnings(False)
        for pin in (left_pwm_pin, left_in1, left_in2, right_pwm_pin, right_in3, right_in4):
            self._gpio.setup(pin, self._gpio.OUT)

        self._left_in1, self._left_in2 = left_in1, left_in2
        self._right_in3, self._right_in4 = right_in3, right_in4

        self._left_pwm = self._gpio.PWM(left_pwm_pin, pwm_freq)
        self._right_pwm = self._gpio.PWM(right_pwm_pin, pwm_freq)
        self._left_pwm.start(0)
        self._right_pwm.start(0)

        self._sub = self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self._watchdog = self.create_timer(0.1, self._watchdog_cb)
        self._last_cmd_time = self.get_clock().now()

        self.get_logger().info('motor_driver_node ready, listening on /cmd_vel')

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()

        v = msg.linear.x
        w = msg.angular.z

        # Differential-drive inverse kinematics
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        self._drive_wheel(v_left, self._left_pwm, self._left_in1, self._left_in2)
        self._drive_wheel(v_right, self._right_pwm, self._right_in3, self._right_in4)

    def _drive_wheel(self, speed, pwm, in_a, in_b):
        # Direction
        if speed >= 0:
            self._gpio.output(in_a, self._gpio.HIGH)
            self._gpio.output(in_b, self._gpio.LOW)
        else:
            self._gpio.output(in_a, self._gpio.LOW)
            self._gpio.output(in_b, self._gpio.HIGH)

        # Speed -> duty cycle. Below max_wheel_speed the mapping is linear,
        # with a floor at min_duty_cycle so slow commands don't stall the
        # motor below its start-up torque threshold.
        magnitude = min(abs(speed), self.max_wheel_speed)
        if magnitude < 1e-3:
            duty = 0.0
        else:
            duty = self.min_duty + (100.0 - self.min_duty) * (magnitude / self.max_wheel_speed)
            duty = min(duty, 100.0)
        pwm.ChangeDutyCycle(duty)

    def _watchdog_cb(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self._left_pwm.ChangeDutyCycle(0)
            self._right_pwm.ChangeDutyCycle(0)

    def destroy_node(self):
        self._left_pwm.stop()
        self._right_pwm.stop()
        self._gpio.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
