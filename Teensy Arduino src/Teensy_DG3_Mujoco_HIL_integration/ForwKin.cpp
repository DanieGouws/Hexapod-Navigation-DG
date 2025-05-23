#include "ForwKin.h"

float L1x = 53.17;
float L1z = 8;
float L2  = 101.88;
float L3  = 149.16;
float B = 125.54;
float rot[6] = {0*M_PI/180, 60*M_PI/180, 120*M_PI/180, 180*M_PI/180, 240*M_PI/180, 300*M_PI/180};

void FK03(float &Px,float &Py,float &Pz,float theta1,float theta2,float theta3)
{
  float C1 = cos(theta1);
  float S1 = sin(theta1);
  float C2 = cos(theta2);
  float S2 = sin(theta2);
  float C23 = cos(theta2 + theta3);
  float S23 = sin(theta2 + theta3);

  //T03 =

  Px = C1*(L1x + L3*C23 + L2*C2);
  Py = S1*(L1x + L3*C23 + L2*C2);
  Pz = L1z + L3*S23 + L2*S2;
}

void FK03_inbody(float &Px,float &Py,float &Pz,float theta1,float theta2,float theta3,int leg)
{
  float x,y,z;
  FK03(x,y,z, theta1,theta2,theta3);

  Px = y*sin(rot[leg]) + (x+B)*cos(rot[leg]);
  Py = y*cos(rot[leg]) - (x+B)*sin(rot[leg]);
  Pz = z;
}
