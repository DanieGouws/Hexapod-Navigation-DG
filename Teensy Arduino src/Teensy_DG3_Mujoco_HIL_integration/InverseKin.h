#ifndef InverseKin_h
#define InverseKin_h

#include "Arduino.h"
#include "interpo.h"

class InverseKinematics
{
    public:
        InverseKinematics();
        void IK(float *theta1,float *theta2,float *theta3,float x,float y,float z,int leg,float dt,float roll=0.0,float pitch=0.0,float yaw=0.0,int inEn=1);
        void difIK(float *theta1Dot,float *theta2Dot,float *theta3Dot,float *theta1,float *theta2,float *theta3,float xDot,float yDot,float zDot,int leg);
    private:
        Interpolation InX1;
        Interpolation InY1;
        Interpolation InZ1;
        Interpolation InYaw1;

        Interpolation InX2;
        Interpolation InY2;
        Interpolation InZ2;
        Interpolation InYaw2;

        Interpolation InX3;
        Interpolation InY3;
        Interpolation InZ3;
        Interpolation InYaw3;

        Interpolation InX4;
        Interpolation InY4;
        Interpolation InZ4;
        Interpolation InYaw4;

        Interpolation InX5;
        Interpolation InY5; 
        Interpolation InZ5;
        Interpolation InYaw5;

        Interpolation InX6;
        Interpolation InY6;
        Interpolation InZ6;
        Interpolation InYaw6;

        float L1x = 53.17;
        float L1z = 8;
        float L2  = 101.88;
        float L3  = 149.16;
        float B = 125.54;
        float rot[6] = {0*M_PI/180, 60*M_PI/180, 120*M_PI/180, 180*M_PI/180, 240*M_PI/180, 300*M_PI/180};
};

#endif
