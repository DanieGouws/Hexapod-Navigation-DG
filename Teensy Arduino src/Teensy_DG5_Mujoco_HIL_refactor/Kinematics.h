#ifndef KINEMATICS_H
#define KINEMATICS_H

// The forward and inverse kinematics for an xyz point in leg frame to servo angles.
// Also forward and inverse transformation from body frame to leg frame for a point.
// Same kinematics as in Sheldon's Thesis.
// All angles in radians with zero as defined by Erasmus kins. Star shaped with all legs point outward straight.
// All xyz coordinates in this lib in mm.
// Some convenience functions to do all 6 points at once? Maybe not.

#include <Arduino.h>

const double L1x = 53.17;
const double L1z = 8;
const double L2 = 101.88;
const double L3 = 149.16;
const double B = 125.54;
const double rot[6] = {0*M_PI/180, 60*M_PI/180, 120*M_PI/180, 180*M_PI/180, 240*M_PI/180, 300*M_PI/180};


class KinsClass {
public:
  void body_to_joint(double *theta1, double *theta2, double *theta3, double x, double y, double z, int leg);
  void joint_to_body(double *x, double *y, double *z, double theta1, double theta2, double theta3, int leg);
private:
  void leg_to_joint(double *theta1, double *theta2, double *theta3, double x, double y, double z);
  void joint_to_leg(double *x, double *y, double *z, double theta1, double theta2, double theta3);
  void leg_to_body(double *xb, double *yb, double *zb, double xl, double yl, double zl, int leg);
  void body_to_leg(double *xl, double *yl, double *zl, double xb, double yb, double zb, int leg);
};

extern KinsClass Kins;

#endif