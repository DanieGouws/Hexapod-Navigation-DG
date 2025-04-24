from numpy import *

class WalkPath:
    def __init__(self,Rd,di,Ss,Sh,Fhs,Fhe,H,rot,p,r):
        self.Rd = Rd
        self.dir = di
        self.Ss = Ss
        self.Sh = Sh
        self.Fhs = Fhs
        self.Fhe = Fhe
        self.H = H
        self.rot = rot
        self.p = p
        self.r = r

    def footplan(self):

        x = array([0.0,0.0,0.0])
        y = array([0.0,0.0,0.0])
        z = array([0.0,0.0,0.0])

        di = self.dir - pi

        dhs = self.Ss/2*(tan(self.p)*cos(di)+tan(self.r)*sin(di)) + self.Fhs
        dhe = -self.Ss/2*(tan(self.p)*cos(di)+tan(self.r)*sin(di)) + self.Fhe

        #Start point
        x[0] = self.Rd*cos(self.rot)+(self.Ss/2)*cos(di)
        y[0] = -(self.Rd*sin(self.rot)+(self.Ss/2)*sin(di))
        z[0] = -self.H + dhs

        #Mid point
        x[1] = self.Rd*cos(self.rot)
        y[1] = -self.Rd*sin(self.rot)
        z[1] = -self.H + self.Sh

        #End point
        x[2] = self.Rd*cos(self.rot)-(self.Ss/2)*cos(di)
        y[2] = -(self.Rd*sin(self.rot)-(self.Ss/2)*sin(di))
        z[2] = -self.H + dhe

        return x,y,z

    def PathPlan(self,x,y,z,TF,PathSize):
        #X-path
        a0x = x[0]
        a1x = 0
        a2x = 0
        a3x = (2/TF**3)*(32*(x[1]-x[0])-11*(x[2]-x[0]))
        a4x = -(3/TF**4)*(64*(x[1]-x[0])-27*(x[2]-x[0]))
        a5x = (3/TF**5)*(64*(x[1]-x[0])-30*(x[2]-x[0]))
        a6x = -(32/TF**6)*(2*(x[1]-x[0])-(x[2]-x[0]))

        #Y-path
        a0y = y[0]
        a1y = 0
        a2y = 0
        a3y = (2/TF**3)*(32*(y[1]-y[0])-11*(y[2]-y[0]))
        a4y = -(3/TF**4)*(64*(y[1]-y[0])-27*(y[2]-y[0]))
        a5y = (3/TF**5)*(64*(y[1]-y[0])-30*(y[2]-y[0]))
        a6y = -(32/TF**6)*(2*(y[1]-y[0])-(y[2]-y[0]))

        a0z = z[0]
        a1z = 0
        a2z = 0
        a3z = (2/TF**3)*(32*(z[1]-z[0])-11*(z[2]-z[0]))
        a4z = -(3/TF**4)*(64*(z[1]-z[0])-27*(z[2]-z[0]))
        a5z = (3/TF**5)*(64*(z[1]-z[0])-30*(z[2]-z[0]))
        a6z = -(32/TF**6)*(2*(z[1]-z[0])-(z[2]-z[0]))

        dt = TF/(PathSize-1)
        t = 0

        xp = zeros((1,PathSize*2-2))
        yp = zeros((1,PathSize*2-2))
        zp = zeros((1,PathSize*2-2))
        xp_d = zeros((1,PathSize*2-2))
        yp_d = zeros((1,PathSize*2-2))
        zp_d = zeros((1,PathSize*2-2))

        #Create path points
        for i in range(PathSize):
            xp[0,i] = a0x + a1x*t + a2x*t**2 + a3x*t**3 + a4x*t**4 + a5x*t**5 + a6x*t**6
            yp[0,i] = a0y + a1y*t + a2y*t**2 + a3y*t**3 + a4y*t**4 + a5y*t**5 + a6y*t**6
            zp[0,i] = a0z + a1z*t + a2z*t**2 + a3z*t**3 + a4z*t**4 + a5z*t**5 + a6z*t**6

            xp_d[0,i] = a1x + 2*a2x*t + 3*a3x*t**2 + 4*a4x*t**3 + 5*a5x*t**4 + 6*a6x*t**5
            yp_d[0,i] = a1y + 2*a2y*t + 3*a3y*t**2 + 4*a4y*t**3 + 5*a5y*t**4 + 6*a6y*t**5
            zp_d[0,i] = a1z + 2*a2z*t + 3*a3z*t**2 + 4*a4z*t**3 + 5*a5z*t**4 + 6*a6z*t**5
            t = t + dt

        j = PathSize-2
        #Points for support phase
        for i in range((PathSize),(PathSize*2-2)):
            xp[0,i] = xp[0,j]
            yp[0,i] = yp[0,j]
            zp[0,i] = zp[0,6]

            xp_d[0,i] = xp_d[0,j]
            yp_d[0,i] = yp_d[0,j]
            zp_d[0,i] = zp_d[0,j]
            j = j - 1
        
        return xp,yp,zp,xp_d,yp_d,zp_d

    def MakePath(self,tf,PathSize):
        
        (x_foot,y_foot,z_foot) = self.footplan()
        (xp,yp,zp,xp_d,yp_d,zp_d) = self.PathPlan(x_foot,y_foot,z_foot,tf,PathSize)

        return xp,yp,zp,xp_d,yp_d,zp_d