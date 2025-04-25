#!/usr/bin/env python3
# need to figure out what this hexapodC does. Init uisng an input Node.
# Create several publishers using fed in node for both input and output of PlannerV1
#

import rclpy
from rclpy.node import Node
from my_hexapod_interfaces.msg import ThetaMessage, LegPath, PathVarNCmdVel
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class HexapodC:
    # Shared class vars
    x = 0.0
    y = 0.0
    t = 0.0
    BH = 140.0
    Ss = 100.0
    Sh = [50.0] * 6
    Fh = [0.0] * 12
    Rd = 283.71
    p = 0.0
    r = 0.0

    def __init__(self, node: Node, ns='/simple_hexapod/'):
        self.node = node
        self.ns = ns
        self.sendpathflag = None
        self.joints = []
        self.angles = None  # Needs to be filled by an external subscriber

        jleg = ['Th1', 'Th2', 'Th3']
        leg = ['1', '2', '3', '4', '5', '6']
        for j in jleg:
            for l in leg:
                self.joints.append(f'{j}_{l}')

        # Publishers
        self.node.get_logger().info('Creating joint command publisher')
        self.pub_angles = self.node.create_publisher(ThetaMessage, ns + 'Th_position_controller/command', 1)
        self.pub_path = self.node.create_publisher(LegPath, ns + 'Legs_paths', 1)
        self.pub_pathDot = self.node.create_publisher(LegPath, ns + 'Legs_pathsDot', 1)
        self._pub_cmd_vel_path_var = self.node.create_publisher(PathVarNCmdVel, ns + 'cmd_vel_path_var', 1)

    #Publish message to "cmd_vel_path_var" about direction and rotation of a step.
    def set_walk_velocity(self, x, y, t):
        HexapodC.x = x
        HexapodC.y = y
        HexapodC.t = t

        msg = PathVarNCmdVel()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = t

        msg.path_var.bh = HexapodC.BH
        msg.path_var.ss = HexapodC.Ss
        msg.path_var.sh = HexapodC.Sh
        msg.path_var.fh = HexapodC.Fh
        msg.path_var.rd = HexapodC.Rd
        msg.path_var.p = HexapodC.p
        msg.path_var.r = HexapodC.r

        self._pub_cmd_vel_path_var.publish(msg)
        self.node.get_logger().info("Published to PathVarNCmdVel")

    def set_path_var(self, BH=140.0, Ss=100.0, Sh=None, Fh=None, Rd=283.71, p=0.0, r=0.0):
        if Sh is None:
            Sh = [50.0] * 6
        if Fh is None:
            Fh = [0.0] * 12

        HexapodC.BH = BH
        HexapodC.Ss = Ss
        HexapodC.Sh = Sh
        HexapodC.Fh = Fh
        HexapodC.Rd = Rd
        HexapodC.p = p
        HexapodC.r = r

        msg = PathVarNCmdVel()
        msg.linear.x = HexapodC.x
        msg.linear.y = HexapodC.y
        msg.angular.z = HexapodC.t
        msg.path_var.BH = BH
        msg.path_var.Ss = Ss
        msg.path_var.Sh = Sh
        msg.path_var.Fh = Fh
        msg.path_var.Rd = Rd
        msg.path_var.p = p
        msg.path_var.r = r

        self._pub_cmd_vel_path_var.publish(msg)

    def get_angles(self):
        if self.joints is None or self.angles is None:
            return None
        return dict(zip(self.joints, self.angles))


    def set_angles(self, th1, th2, th3):
        msg = ThetaMessage()
        for i in range(6):
            setattr(msg, f'th1_{i+1}', th1[i])
            setattr(msg, f'th2_{i+1}', th2[i])
            setattr(msg, f'th3_{i+1}', th3[i])
        self.pub_angles.publish(msg)

    #publish points in a step. All points over the perabula
    def set_path(self, xpath, ypath, zpath, turnpath, xpathDot, ypathDot, zpathDot, turnpathDot, Dt):
        if self.sendpathflag == 1:
            msg = LegPath()
            msg_dot = LegPath()

            for i in range(6):
                setattr(msg, f'path_l{i}_x', list(xpath[i, :]))    #path_l0_x
                setattr(msg_dot, f'path_l{i}_x', list(xpathDot[i, :]))

                setattr(msg, f'path_l{i}_y', list(ypath[i, :]))
                setattr(msg_dot, f'path_l{i}_y', list(ypathDot[i, :]))

                setattr(msg, f'path_l{i}_z', list(zpath[i, :]))
                setattr(msg_dot, f'path_l{i}_z', list(zpathDot[i, :]))

            msg.path_ang = list(turnpath[0, :])
            msg_dot.path_ang = list(turnpathDot[0, :])
            msg.dt = Dt

            self.pub_path.publish(msg)
            self.pub_pathDot.publish(msg_dot)
            self.sendpathflag = 0
