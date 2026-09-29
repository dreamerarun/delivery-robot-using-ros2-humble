#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomCovRelay(Node):
    def __init__(self):
        super().__init__('odom_cov_relay')
        self.sub = self.create_subscription(Odometry, '/odom_rf2o', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom_rf2o_cov', 10)

    def cb(self, msg):
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.02
        pose_cov[7] = 0.02
        pose_cov[35] = 0.05

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.02
        twist_cov[7] = 0.02
        twist_cov[35] = 0.05

        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
