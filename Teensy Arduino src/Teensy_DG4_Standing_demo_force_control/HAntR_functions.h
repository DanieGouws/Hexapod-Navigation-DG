#ifndef HAntR_functions_h
#define HAntR_functions_h

#include <ArduinoEigen.h>
using namespace Eigen;

//walking demo
const float STEP_SIZE = 50.0f;     // 10 cm per step
const float SQUARE_SIZE = 500.0f;   // 0.5 meter square
const int NUM_STEPS = 4 * (int)(SQUARE_SIZE / STEP_SIZE);  // 40 steps

class Poly6Interpolator3D {
  public:
    Poly6Interpolator3D();
    void set(Vector3f p0, Vector3f pmid, Vector3f p1, float tf);
    Vector3f get(float t) const;

  private:
    float a[3][7];  // Coefficients for x, y, z
    float tf_;      // Total time
};


// void SetAngles(float* th1,float* th2,float* th3 ,float spd1,float spd2, float spd3);
Matrix4f computeBodyPose(Vector3f foot_tips[6], Vector3f gravity);
Vector3f world_to_body_kins(Vector3f foot_tips_world, Matrix4f body_pose);
void copyFeet(Vector3f dest[6], Vector3f src[6]);
Vector2f getSquareStepPoint(int index);
void offsetFeet(Vector3f dest[6], const Vector3f src[6], const Vector2f& offset);


#endif