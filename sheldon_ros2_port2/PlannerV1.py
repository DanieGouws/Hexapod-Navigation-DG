#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from threading import Thread
from math import atan2, sqrt
from numpy import zeros
import time

from my_hexapod_interfaces.msg import PathVarNCmdVel  # Adjust if needed
from sheldon_ros2_port2.hexapodC import HexapodC
from sheldon_ros2_port2.movefunc import MoveFunc

class Planner(Node):
    def __init__(self, robot, BH=140.0, Ss=100.0, Sh=50.0, Rd=283.71, p=0.0, r=0.0):
        super().__init__('planner')

        self.robot = robot
        self.BH = BH
        self.Ss = Ss
        self.Sh = Sh
        self.Rd = Rd
        self.p = p
        self.r = r
        self.di = 0.0
        self.walk_vel = 0.0
        self.Move = MoveFunc()

        self.pathsize = 7
        self.ang = zeros((1, self.pathsize * 2 - 1))

        self._th_walk = None
        self.running = False
        self.walking = False

        self.subscription = self.create_subscription(
            PathVarNCmdVel,
            robot.ns + 'cmd_vel_path_var',
            self._cb_cmdvel_pathVar,
            1
        )


    # Commands from some higher level planner which just gives target direction and step parameters.
    def _cb_cmdvel_pathVar(self, msg):
        self.robot.node.get_logger().info('Received cmd_vel and path_var')
        print("Here")

        vx = msg.linear.x
        vy = msg.linear.y
        self.yaw = msg.angular.z

        self.BH = msg.path_var.bh
        self.Ss = msg.path_var.ss
        self.Sh = msg.path_var.sh
        self.Fh = msg.path_var.fh
        self.Rd = msg.path_var.rd
        self.p = msg.path_var.p
        self.r = msg.path_var.r

        self.di = atan2(vy, vx)
        self.walk_vel = sqrt(vx ** 2 + vy ** 2)

        if self.walk_vel != 0:
            tf = self.Ss / (self.walk_vel * 1000)
            self.dt = tf / (self.pathsize - 1)
            self.x, self.y, self.z, self.xd, self.yd, self.zd = self.Move.makepath_walk(
                self.Ss, self.Sh, self.Fh, self.Rd, self.p, self.r, self.di, self.BH, tf, self.pathsize)
            self.ang, _, self.ang_d, _ = self.Move.makepath_turn(self.yaw, 0, tf, self.pathsize)
        elif self.walk_vel == 0 and self.yaw != 0:
            tf = 2
            self.dt = tf / (self.pathsize - 1)
            self.x, self.y, self.z, self.xd, self.yd, self.zd = self.Move.makepath_walk(
                0, self.Sh, self.Fh, self.Rd, self.p, self.r, self.di, self.BH, tf, self.pathsize)
            self.ang, _, self.ang_d, _ = self.Move.makepath_turn(self.yaw, 0, tf, self.pathsize)
        else:
            tf = 2
            self.dt = -1
            self.x, self.y, self.z, self.xd, self.yd, self.zd = self.Move.makepath_walk(
                0, self.Sh, self.Fh, self.Rd, self.p, self.r, self.di, self.BH, tf, self.pathsize)
            self.ang, _, self.ang_d, _ = self.Move.makepath_turn(0, 0, tf, self.pathsize)

        self.robot.sendpathflag = 1
        self.start()

    def start(self):
        if not self.running:
            self.running = True
            self._th_walk = Thread(target=self.do_walk)
            self._th_walk.start()
            self.walking = True

    def stop(self):
        if self.running:
            self.walking = False
            self.get_logger().info('Waiting for stopped')
            if self._th_walk is not None:
                self._th_walk.join()
            self.get_logger().info('Stopped')
            self.running = False

    def do_walk(self):
        self.get_logger().info('Started walking thread')
        self.Move.prevtime = time.time() * 1000
        pretime = time.time() * 1000

        while rclpy.ok() and self.walking:
            curtime = time.time() * 1000
            if curtime - pretime >= 10:
                pretime = curtime
                self.robot.set_path(
                    self.x, self.y, self.z, self.ang,
                    self.xd, self.yd, self.zd, self.ang_d,
                    self.dt * 1000
                )

def main(args=None):
    rclpy.init(args=args)
    dummy_node = Node("PlannerV1_node")\
    
    rclpy.logging.get_logger("planner").info("Instantiating hexapod Client")
    robot = HexapodC(dummy_node)
    planner = Planner(robot)
    rclpy.logging.get_logger("planner").info("Planner Ready")

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.stop()
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
