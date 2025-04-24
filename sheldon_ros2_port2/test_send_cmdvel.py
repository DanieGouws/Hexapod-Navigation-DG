#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from my_hexapod_interfaces.msg import PathVarNCmdVel

class HexapodTestPublisher(Node):
    def __init__(self):
        super().__init__('hexapod_test_publisher')

        self.publisher_ = self.create_publisher(
            PathVarNCmdVel,
            '/simple_hexapod/cmd_vel_path_var',
            10
        )

        # Publish after short delay
        self.timer = self.create_timer(1.0, self.publish_once)
        self.has_published = False

    def publish_once(self):
        if self.has_published:
            return

        msg = PathVarNCmdVel()
        msg.linear.x = 0.1
        msg.linear.y = 0.0
        msg.angular.z = 0.2

        msg.path_var.bh = 140.0
        msg.path_var.ss = 100.0
        msg.path_var.sh = [50.0] * 6
        msg.path_var.fh = [0.0] * 12
        msg.path_var.rd = 283.71
        msg.path_var.p = 0.0
        msg.path_var.r = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info('Published one PathVarNCmdVel message')

        self.has_published = True
        self.timer.cancel()  # Stop the timer after one publish


def main(args=None):
    rclpy.init(args=args)
    node = HexapodTestPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
