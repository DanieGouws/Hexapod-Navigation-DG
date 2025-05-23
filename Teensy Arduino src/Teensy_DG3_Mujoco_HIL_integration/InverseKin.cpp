#include "InverseKin.h"

InverseKinematics::InverseKinematics(): InX1(429.75),InY1(0.0),InZ1(8.0),InYaw1(0.0),InX2(214.875),InY2(-372.174),InZ2(8.0),InYaw2(0.0),InX3(-214.875),InY3(-372.174),InZ3(8.0),InYaw3(0.0),InX4(-429.75),InY4(0.0),InZ4(8.0),InYaw4(0.0),InX5(-214.875),InY5(372.174),InZ5(8.0),InYaw5(0.0),InX6(214.875),InY6(372.174),InZ6(8.0),InYaw6(0.0)
//InX1(283.71),InY1(0.0),InZ1(-140),InYaw1(0.0),InX2(141.855),InY2(-245.7),InZ2(-140),InYaw2(0.0),InX3(-141.855),InY3(-245.7),InZ3(-140),InYaw3(0.0),InX4(-283.71),InY4(0.0),InZ4(-140),InYaw4(0.0),InX5(-141.855),InY5(245.7),InZ5(-140),InYaw5(0.0),InX6(141.855),InY6(245.7),InZ6(-140),InYaw6(0.0)
{

}

//roll, pitch yaw angles in degrees, output theta 1-3 in rads
void InverseKinematics::IK(float *theta1,float *theta2,float *theta3,float x,float y,float z,int leg,float dt,float roll,float pitch,float yaw,int inEn) //can allow for theta offset
{
    
	float xp=0;
	float yp=0;
	float zp=0;
	float yawp=0;

	float yaw_rad;
	float existingAngle;
	float radius;
	float demandYaw;
	float xxp;
	float yyp;

	float zzp;

	float xL;
	float yL;
	float zL;

	float C1;
	float S1;
	float C3;
	float C3_temp;
	float S3;

	if (leg == 0)
	{
	    xp = InX1.go(x,dt);
	    yp = InY1.go(y,dt);
	    zp = InZ1.go(z,dt);
	    yawp = InYaw1.go(yaw,dt);
	}
	if (leg == 1)
	{
	    xp = InX2.go(x,dt);
	    yp = InY2.go(y,dt);
	    zp = InZ2.go(z,dt);
	    yawp = InYaw2.go(-yaw,dt);
	}
	if (leg == 2)
	{
	    xp = InX3.go(x,dt);
	    yp = InY3.go(y,dt);
	    zp = InZ3.go(z,dt);
	    yawp = InYaw3.go(yaw,dt);
	}
	if (leg == 3)
	{
	    xp = InX4.go(x,dt);
	    yp = InY4.go(y,dt);
	    zp = InZ4.go(z,dt);
	    yawp = InYaw4.go(-yaw,dt);
	}
	if (leg == 4)
	{
	    xp = InX5.go(x,dt);
	    yp = InY5.go(y,dt);
	    zp = InZ5.go(z,dt);
	    yawp = InYaw5.go(yaw,dt);
	}
	if (leg == 5)
	{
	    xp = InX6.go(x,dt);
	    yp = InY6.go(y,dt);
	    zp = InZ6.go(z,dt);
	    yawp = InYaw6.go(-yaw,dt);
	}


	// ** YAW CALCS **

	//degree to radians
	yaw_rad = yawp * M_PI/180.0;

	//calc existing yaw angle
	existingAngle = atan2(yp,xp);

	//calc radius from centre
	radius = sqrt(pow(xp,2)+pow(yp,2));

	//calc demand yaw angle
	demandYaw = existingAngle + yaw_rad;

	//calc new x and y based on demand yaw
	xxp = radius*cos(demandYaw);
	yyp = radius*sin(demandYaw);

	//calc new z based on demand pitch and roll
	zzp = zp + xxp*tan(pitch*M_PI/180.0)+yyp*tan(roll*M_PI/180.0);

	//Transform body to leg coordinate
	xL = xxp*cos(rot[leg]) - yyp*sin(rot[leg]) - B;
	yL = xxp*sin(rot[leg]) + yyp*cos(rot[leg]);
	zL = zzp;

	*theta1 = atan2(yL,xL);

	C1  = cos(*theta1);
	S1  = sin(*theta1);

	C3_temp = ( pow(( xL - L1x*C1 ),2) + pow(( yL - L1x*S1 ),2) + pow(( zL - L1z ),2) - pow(L2,2) - pow(L3,2) )/( 2*L2*L3 );

	if (C3_temp>1)
	{
		C3 = 1;
	}
	else if (C3_temp<-1)
	{
		C3 = -1;
	}
	else
	{
		C3 = C3_temp;
	}

	S3 = -sqrt((1 - pow(C3,2)));

	*theta2 = atan2( zL-L1z, sqrt( pow((xL-L1x*C1),2) + pow((yL - L1x*S1),2) ) ) - atan2(L3*S3, L2 + L3*C3);

	*theta3 = atan2(S3, C3);
  

}


void InverseKinematics::difIK(float *theta1Dot,float *theta2Dot,float *theta3Dot,float *theta1,float *theta2,float *theta3,float xDot,float yDot,float zDot,int leg)
{
  float th1 = *theta1;
  float th2 = *theta2;
  float th3 = *theta3;

  float xd_leg = xDot/1000.0*cos(rot[leg]) - yDot/1000.0*sin(rot[leg]);
  float yd_leg = xDot/1000.0*sin(rot[leg]) + yDot/1000.0*cos(rot[leg]);
  float zd_leg = zDot/1000.0;

  float l1x = L1x/1000;
  //float l1z = L1z;
  float l2 = L2/1000;
  float l3 = L3/1000;

  *theta1Dot = (yd_leg*cos(th1) - xd_leg*sin(th1))/(l1x + l3*cos(th2 + th3) + l2*cos(th2));
  *theta2Dot = (1/l2)*( zd_leg*cos(th2) - xd_leg*cos(th1)*sin(th2) - yd_leg*sin(th1)*sin(th2) + ((cos(th3))/(sin(th3)))*(zd_leg*sin(th2) + xd_leg*cos(th1)*cos(th2) + yd_leg*cos(th2)*sin(th1)) );
  *theta3Dot = -(1/l2)*( zd_leg*cos(th2) - xd_leg*cos(th1)*sin(th2) - yd_leg*sin(th1)*sin(th2) + (l2 + l3*(cos(th3))/(l3*sin(th3)))*(zd_leg*sin(th2) + xd_leg*cos(th1)*cos(th2) + yd_leg*cos(th2)*sin(th1)) );

  *theta1Dot = *theta1Dot*60.0/(2.0*M_PI);
  *theta2Dot = *theta2Dot*60.0/(2.0*M_PI);
  *theta3Dot = *theta3Dot*60.0/(2.0*M_PI);

  if(isnan(*theta1Dot))
  {
    *theta1Dot = 25;
  }
  else if(*theta1Dot < 1)
  {
    *theta1Dot = 1;
  }
  else if(*theta1Dot > 53.2)
  {
    *theta1Dot = 53.2;
  }

  if(isnan(*theta2Dot))
  {
    *theta2Dot = 25;
  }
  else if(*theta2Dot < 1)
  {
    *theta2Dot = 1;
  }
  else if(*theta2Dot > 53.2)
  {
    *theta2Dot = 53.2;
  }

  if(isnan(*theta3Dot))
  {
    *theta3Dot = 25;
  }
  else if(*theta3Dot < 1)
  {
    *theta3Dot = 1;
  }
  else if(*theta3Dot > 53.2)
  {
    *theta3Dot = 53.2;
  }
  
}
