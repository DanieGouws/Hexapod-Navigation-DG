#include "Kinematics.h"

void KinsClass::body_to_joint(double *theta1, double *theta2, double *theta3, double x, double y, double z, int leg) {
  double xl, yl, zl;
  body_to_leg(&xl, &yl, &zl, x, y, z, leg);
  leg_to_joint(theta1, theta2, theta3, xl, yl, zl);
}

void KinsClass::joint_to_body(double *x, double *y, double *z, double theta1, double theta2, double theta3, int leg) {
  double xl, yl, zl;
  joint_to_leg(&xl, &yl, &zl, theta1, theta2, theta3);
  leg_to_body(x, y, z, xl, yl, zl, leg);
}

void KinsClass::joint_to_leg(double *x, double *y, double *z, double theta1, double theta2, double theta3) {
  double C1 = cos(theta1);
  double S1 = sin(theta1);
  double C2 = cos(theta2);
  double S2 = sin(theta2);
  double C23 = cos(theta2 + theta3);
  double S23 = sin(theta2 + theta3);

  *x = C1 * (L1x + L3 * C23 + L2 * C2);
  *y = S1 * (L1x + L3 * C23 + L2 * C2);
  *z = L1z + L3 * S23 + L2 * S2;
}

void KinsClass::leg_to_joint(double *theta1, double *theta2, double *theta3, double x, double y, double z) {
  *theta1 = atan2(y, x);
  double C1 = cos(*theta1);
  double S1 = sin(*theta1);
  double C3 = (pow((x - L1x * C1), 2) + pow((y - L1x * S1), 2) + pow((z - L1z), 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3);
  double S3 = -sqrt((1 - pow(C3, 2)));
  *theta2 = atan2(z - L1z, sqrt(pow((x - L1x * C1), 2) + pow((y - L1x * S1), 2))) - atan2(L3 * S3, L2 + L3 * C3);  //L1x Error in Sheldon's thesis. Correct here and in Christopher.
  C3 = constrain(C3, -1.0, 1.0);                                                                                   //Sheldon does it.
  *theta3 = atan2(S3, C3);
}

void KinsClass::leg_to_body(double *xb, double *yb, double *zb, double xl, double yl, double zl, int leg) {
  *xb = yl * sin(rot[leg]) + (xl + B) * cos(rot[leg]);
  *yb = yl * cos(rot[leg]) - (xl + B) * sin(rot[leg]);
  *zb = zl;
}

void KinsClass::body_to_leg(double *xl, double *yl, double *zl, double xb, double yb, double zb, int leg) {
  *xl = xb * cos(rot[leg]) - yb * sin(rot[leg]) - B;
  *yl = xb * sin(rot[leg]) + yb * cos(rot[leg]);
  *zl = zb;
}

KinsClass Kins = KinsClass();
