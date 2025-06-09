#ifndef POLY6INTERPOLATOR3D_H
#define POLY6INTERPOLATOR3D_H

// Class to create a 6th order polynomial between any 3 points in 3D space such that it passes 
// through the points and has 0 velocity and acceleration at the start and end points. Can be sampled as a continuos function.
// Meant for modeling foot tip path of hexapod robot through 1 step.

#include <ArduinoEigen.h>  //Convenient use of vectors
using namespace Eigen;

class Poly6Interpolator3D {
  public:
    Poly6Interpolator3D();
    void set(Vector3f p0, Vector3f pmid, Vector3f p1, double tf = 1.0);
    Vector3f get(double t) const;

  private:
    double a[3][7];  // Coefficients for x, y, z
    double tf_;      // Total time
};

#endif