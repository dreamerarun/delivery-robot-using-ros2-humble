#!/usr/bin/env python3
"""Starts and stops mapping.launch.py / navigation.launch.py on request, so the
console can trigger them with a button instead of you typing `ros2 launch`.
Also owns ekf.launch.py: starts it immediately on startup, restarts it if it
dies, and makes sure it's alive before bringing up mapping or navigation
(both need `odom -> base_link` from it).

bringup.launch.py's own include of ekf.launch.py isn't resolving on this
setup (that's the "does not exist" you saw), so EKF is managed here instead
-- don't also try to fix the include in bringup.launch.py, or you'll be back
to two rf2o_laser_odometry nodes fighting each other.

Run this ALONGSIDE rosbridge (see console_launch.py), on whichever machine
you currently run `ros2 launch amr_navigation mapping.launch.py` /
`navigation.launch.py` / `ekf.launch.py` from.

Listens on:   /amr/mode_cmd    (std_msgs/String)
    "idle"                        -> stop mapping/navigation (EKF keeps running)
    "mapping"                     -> stop current, then mapping.launch.py
    "navigation:/abs/path/map.yaml" -> stop current, then navigation.launch.py map:=...
Publishes on: /amr/mode_status (std_msgs/String)
    "idle" | "starting_mapping" | "mapping" | "starting_navigation" |
    "navigation" | "stopping" | "error: <reason>"
"""
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LaunchManager(Node):
    def __init__(self):
        super().__init__('launch_manager_node')
        self.proc = None
        self.mode = 'idle'
        self.ekf_proc = None
        self.ekf_stopping = False
        self.lock = threading.Lock()
        self.status_pub = self.create_publisher(String, '/amr/mode_status', 10)
        self.create_subscription(String, '/amr/mode_cmd', self.on_cmd, 10)
        # Republish current status periodically so a console that (re)connects
        # late still finds out what's running.
        self.create_timer(5.0, lambda: self.publish_status(self.mode))
        self.create_timer(2.0, self.watch_ekf)
        self.start_ekf()
        self.publish_status('idle')
        self.get_logger().info('Launch manager ready: idle / mapping / navigation:<map.yaml>')

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f'status: {text}')

    def start_ekf(self):
        self.get_logger().info('Starting EKF (ekf.launch.py)...')
        self.ekf_proc = subprocess.Popen(
            ['ros2', 'launch', 'amr_navigation', 'ekf.launch.py'], preexec_fn=os.setsid)

    def watch_ekf(self):
        if self.ekf_stopping:
            return
        if self.ekf_proc is None or self.ekf_proc.poll() is not None:
            self.get_logger().warn('EKF is not running (or died) -- restarting it.')
            self.start_ekf()

    def ensure_ekf(self):
        if self.ekf_proc is None or self.ekf_proc.poll() is not None:
            self.start_ekf()
            time.sleep(3.0)  # give odom -> base_link a moment to appear before Nav2/SLAM start

    def on_cmd(self, msg: String):
        # Do the actual work off the subscription callback so a slow launch
        # doesn't block message processing.
        threading.Thread(target=self.handle_cmd, args=(msg.data.strip(),), daemon=True).start()

    def handle_cmd(self, cmd: str):
        with self.lock:
            if cmd.startswith('navigation:'):
                mode, map_path = 'navigation', cmd.split(':', 1)[1].strip()
            else:
                mode, map_path = cmd, None

            if mode not in ('idle', 'mapping', 'navigation'):
                self.publish_status(f'error: unknown mode "{cmd}"')
                return
            if mode == 'navigation' and not map_path:
                self.publish_status('error: navigation needs a map path')
                return
            if mode == self.mode and mode != 'idle':
                self.publish_status(mode)  # already there
                return

            self.stop_current()

            if mode == 'idle':
                self.mode = 'idle'
                self.publish_status('idle')
                return

            self.ensure_ekf()
            self.publish_status('starting_' + mode)
            if mode == 'mapping':
                args = ['ros2', 'launch', 'amr_navigation', 'mapping.launch.py']
            else:
                args = ['ros2', 'launch', 'amr_navigation', 'navigation.launch.py', f'map:={map_path}']

            # New process group so we can kill the whole launch tree, not just
            # the `ros2 launch` wrapper process.
            self.proc = subprocess.Popen(args, preexec_fn=os.setsid)
            self.mode = mode
            time.sleep(3.0)  # let it fail fast if the launch file itself is broken
            if self.proc.poll() is not None:
                self.publish_status(f'error: {mode} exited immediately (code {self.proc.returncode})')
                self.mode = 'idle'
                self.proc = None
            else:
                self.publish_status(mode)

    def stop_current(self):
        if self.proc is None:
            return
        self.publish_status('stopping')
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=8)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self.proc = None

    def stop_ekf(self):
        self.ekf_stopping = True
        if self.ekf_proc is None:
            return
        try:
            os.killpg(os.getpgid(self.ekf_proc.pid), signal.SIGINT)
            self.ekf_proc.wait(timeout=8)
        except Exception:
            try:
                os.killpg(os.getpgid(self.ekf_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self.ekf_proc = None


def main():
    rclpy.init()
    node = LaunchManager()
    try:
        rclpy.spin(node)
    finally:
        node.stop_current()
        node.stop_ekf()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
