#ifndef ForwKin_h
#define ForwKin_h

#include "Arduino.h"

void FK03(float &Px,float &Py,float &Pz,float theta1,float theta2,float theta3);
void FK03_inbody(float &Px,float &Py,float &Pz,float theta1,float theta2,float theta3,int leg);

#endif
