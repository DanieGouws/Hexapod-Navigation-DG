#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from numpy import zeros
from math import atan2, sqrt

from my_hexapod_interfaces.msg import PathVarNCmdVel, LegPath
from sheldon_ros2_port2.movefunc import MoveFunc


class Planner(Node):
    def __init__(self):
        super().__init__('planner')

        self.ns = '/simple_hexapod/'
        self.declare_parameter('publish_rate_hz', 100.0)
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            PathVarNCmdVel,
            self.ns + 'cmd_vel_path_var',
            self._cb_cmdvel_pathVar,
            10
        )

        self.pub_path = self.create_publisher(LegPath, self.ns + 'Legs_paths', 10)
        self.pub_pathDot = self.create_publisher(LegPath, self.ns + 'Legs_pathsDot', 10)

        self.Move = MoveFunc()
        self.pathsize = 7
        self.ang = zeros((1, self.pathsize * 2 - 1))

        self.dt = None
        self.data_ready = False

        # Timer for continuous publishing
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.prev_time = time.time() * 1000  # For dt tracking if needed

        self.get_logger().info("Planner node initialized")

    def _cb_cmdvel_pathVar(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        yaw = msg.angular.z

        BH = msg.path_var.bh
        Ss = msg.path_var.ss
        Sh = msg.path_var.sh
        Fh = msg.path_var.fh
        Rd = msg.path_var.rd
        p = msg.path_var.p
        r = msg.path_var.r

        di = atan2(vy, vx)
        walk_vel = sqrt(vx ** 2 + vy ** 2)

        if walk_vel != 0:
            tf = Ss / (walk_vel * 1000)
        else:
            tf = 2.0

        self.dt = tf / (self.pathsize - 1)

        self.x, self.y, self.z, self.xd, self.yd, self.zd = self.Move.makepath_walk(
            Ss if walk_vel != 0 else 0, Sh, Fh, Rd, p, r, di, BH, tf, self.pathsize
        )

        self.ang, _, self.ang_d, _ = self.Move.makepath_turn(yaw, 0, tf, self.pathsize)

        self.data_ready = True
        self.get_logger().info("Received motion data and calculated path")

    def _timer_cb(self):
        if not self.data_ready:
            return

        msg = LegPath()
        msg_dot = LegPath()

        for i in range(6):
            setattr(msg, f'path_l{i}_x', list(self.x[i, :]))
            setattr(msg_dot, f'path_l{i}_x', list(self.xd[i, :]))

            setattr(msg, f'path_l{i}_y', list(self.y[i, :]))
            setattr(msg_dot, f'path_l{i}_y', list(self.yd[i, :]))

            setattr(msg, f'path_l{i}_z', list(self.z[i, :]))
            setattr(msg_dot, f'path_l{i}_z', list(self.zd[i, :]))

        msg.path_ang = list(self.ang[0, :])
        msg_dot.path_ang = list(self.ang_d[0, :])
        msg.dt = self.dt * 1000  # milliseconds

        self.pub_path.publish(msg)
        self.pub_pathDot.publish(msg_dot)
        self.get_logger().info("Started publishing path messages")

        # Optionally set data_ready = False if you want one-time publishing
        self.data_ready = False 


def main(args=None):
    rclpy.init(args=args)
    node = Planner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
