#ifndef HANTR_FUNCTIONS_H
#define HANTR_FUNCTIONS_H

// Functions specifically for implementing the HANTR Hexapod controller concept. The user should handle
// the state machine and generation of foot points in world frame. This file then calculates a reasonable
// body pose in the world frame. The supplied foot coordinates in world frame and body pose
// are then converted to points in the body frame. User should then use kinematics to convert these points
// to servo commands.

#include <ArduinoEigen.h>  //Convenient use of vectors
using namespace Eigen;

class HantrClass {
public:
  Matrix4f computeBodyPose(Vector3f foot_tips[6], Vector3f gravity);
  Vector3f world_to_body_kins(Vector3f foot_tips_world, Matrix4f body_pose);
  void offsetOrCopyFeet(Vector3f dest[6], const Vector3f src[6], const Vector2f offset = Vector2f(0, 0));
};

extern HantrClass Hantr;

#endif