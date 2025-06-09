#include "Poly6Interpolator3D.h"

Poly6Interpolator3D::Poly6Interpolator3D(){}

// Initialize with start, mid, end points and total duration
void Poly6Interpolator3D::set(Vector3f p0, Vector3f pmid, Vector3f p1, double tf) {
  tf_ = tf;

  for (int i = 0; i < 3; ++i) {
    double s0 = p0[i];
    double sm = pmid[i];
    double s1 = p1[i];

    double dx1 = sm - s0;
    double dx2 = s1 - s0;

    // Coefficients: a0 + a1*t + ... + a6*t^6
    a[i][0] = s0;
    a[i][1] = 0;
    a[i][2] = 0;
    a[i][3] = (2 / pow(tf, 3)) * (32 * dx1 - 11 * dx2);
    a[i][4] = -(3 / pow(tf, 4)) * (64 * dx1 - 27 * dx2);
    a[i][5] = (3 / pow(tf, 5)) * (64 * dx1 - 30 * dx2);
    a[i][6] = -(32 / pow(tf, 6)) * (2 * dx1 - dx2);
  }
}

// Get position at time t ∈ [0, tf]
Vector3f Poly6Interpolator3D::get(double t) const {
  t = constrain(t, 0.0f, tf_);
  Vector3f p;
  for (int i = 0; i < 3; ++i) {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    double t6 = t5 * t;
    p[i] = a[i][0] + a[i][1]*t + a[i][2]*t2 + a[i][3]*t3 +
            a[i][4]*t4 + a[i][5]*t5 + a[i][6]*t6;
  }
  return p;
}