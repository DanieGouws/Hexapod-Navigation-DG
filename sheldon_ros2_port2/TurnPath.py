from numpy import *

class TurnPath:
    def __init__(self,Angstart,Zstart,AngSs,ZSs):
        self.Angstart = Angstart
        self.Zstart = Zstart
        self.AngSs = AngSs
        self.ZSs = ZSs

    def footplan(self):

        x = array([0.0,0.0,0.0])
        y = array([0.0,0.0,0.0])

        #Start point
        x[0] = self.Angstart
        y[0] = self.Zstart

        #Mid point
        x[1] = self.AngSs/2.0
        y[1] = self.ZSs

        #End point
        x[2] = self.AngSs
        y[2] = self.Zstart

        return x,y

    def PathPlan(self,x,y,TF,PathSize):
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

        dt = TF/(PathSize-1)
        t = 0

        xp = zeros((1,PathSize*2-2))
        yp = zeros((2,PathSize*2-2))
        xp_d = zeros((1,PathSize*2-2))
        yp_d = zeros((2,PathSize*2-2))

        #Create path points
        for i in range(PathSize):
            xp[0,i] = a0x + a1x*t + a2x*t**2 + a3x*t**3 + a4x*t**4 + a5x*t**5 + a6x*t**6
            yp[0,i] = a0y + a1y*t + a2y*t**2 + a3y*t**3 + a4y*t**4 + a5y*t**5 + a6y*t**6
            yp[1,i] = yp[0,0]

            xp_d[0,i] = a1x + 2*a2x*t + 3*a3x*t**2 + 4*a4x*t**3 + 5*a5x*t**4 + 6*a6x*t**5
            yp_d[0,i] = a1y + 2*a2y*t + 3*a3y*t**2 + 4*a4y*t**3 + 5*a5y*t**4 + 6*a6y*t**5
            yp_d[1,i] = yp_d[0,0]
            t = t + dt

        j = PathSize-2
        #Points for support phase
        for i in range((PathSize),(PathSize*2-2)):
            xp[0,i] = xp[0,j]
            # yp[0,i] = yp[0,6]
            # yp[1,i] = yp[0,j]

            xp_d[0,i] = xp_d[0,j]
            # yp_d[0,i] = yp_d[0,6]
            # yp_d[1,i] = yp_d[0,j]
            j = j - 1
        
        return xp,yp,xp_d,yp_d

    def MakePath(self,tf,PathSize):

        (x_foot,y_foot) = self.footplan()
        (xp,yp,xp_d,yp_d) = self.PathPlan(x_foot,y_foot,tf,PathSize)

        return xp,yp,xp_d,yp_d
