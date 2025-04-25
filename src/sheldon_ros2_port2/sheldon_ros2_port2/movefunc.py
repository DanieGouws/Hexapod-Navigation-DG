from math import pi,sin,cos
#from IK import *
#from FK import *
from sheldon_ros2_port2 import TurnPath
from sheldon_ros2_port2 import WalkPath
from numpy import zeros
import time

class MoveFunc:
    def __init__(self):
        #Constant variables
        self.B = 125.54
        self.rot = [0*pi/180, 60*pi/180, 120*pi/180, 180*pi/180, 240*pi/180, 300*pi/180]

        #Global variables
        self.currentPathPoint = [3,9,3,9,3,9]; self.currentPathPoint_w = [3,9]; self.currentPathPoint_t = [0,0]; self.currentPathPoint_tw = [3,3]
        self.PosX = zeros((6,)); self.PosY = zeros((6,)); self.PosZ = zeros((6,))
        self.theta1 = zeros((6,)); self.theta2 = zeros((6,)); self.theta3 = zeros((6,)); self.OG_theta1 = zeros((6,)); self.OG_PosZ = zeros((6,))
        self.theta1_d = zeros((6,)); self.theta2_d = zeros((6,)); self.theta3_d = zeros((6,))
        self.bodyTwist = 0.0
        self.mode = ""
        self.legRotation = 0.0
        self.prevtime = 0
        self.currenttime = 0

    def Leg2body(self,x,y,rot):
        XG = y*sin(rot)+(x+self.B)*cos(rot)
        YG = y*cos(rot)-(x+self.B)*sin(rot)
        return XG,YG

    def Body2leg(self,x,y,rot):
        XL = x*cos(rot) - y*sin(rot) - self.B
        YL = x*sin(rot) + y*cos(rot)
        return XL,YL

    def makepath_walk(self,Ss,Sh,Fh,Rd,p,r,di,BH,tf = 2,pathsize = 7):
        WP = WalkPath.WalkPath(Rd,di,Ss,Sh[0],Fh[0],Fh[6],BH,self.rot[0],p,r)
        xp = zeros((6,7*2-2)); yp = zeros((6,7*2-2)); zp = zeros((6,7*2-2))
        xp_d = zeros((6,7*2-2)); yp_d = zeros((6,7*2-2)); zp_d = zeros((6,7*2-2))
        for i in range(6):
            WP.rot = self.rot[i]
            WP.Sh = Sh[i]
            WP.Fhs = Fh[i]
            WP.Fhe = Fh[i+6]
            (xp[i,],yp[i,],zp[i,],xp_d[i,],yp_d[i,],zp_d[i,]) = WP.MakePath(tf,pathsize)
        
        return xp,yp,zp,xp_d,yp_d,zp_d

    def makepath_turn(self,AngSS,ZSS,tf = 2,pathsize = 7):
        TP = TurnPath.TurnPath(0,0,AngSS,ZSS)
        (ang,z,ang_d,z_d) = TP.MakePath(tf,pathsize)
        
        return ang,z,ang_d,z_d