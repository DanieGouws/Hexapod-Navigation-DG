#include "HAntR_functions.h"

// Get Body pos and orientation as a 4x4 mat. 
// Generates a "nice" pos for the body frame in world space given the pos of all the feet in world frame.
// Body is const distance above feet centroid, pitch and roll level, yaw aimed in average feet direction.
// Need a more general implementation to get other body pos and orientation.
Matrix4f HantrClass::computeBodyPose(Vector3f foot_tips[6], Vector3f gravity) {
  
  // Step 1: Compute centroid of foot tips
  Vector3f centroid(0, 0, 0);
  for (int i = 0; i < 6; ++i) {
    centroid += foot_tips[i];
  }
  centroid /= 6.0;

  // Step 2: Normalize gravity vector
  Vector3f ghat = gravity.normalized();

  // Step 3: Compute body translation
  const double h_def = 150.0;  // Default body height above feet centroid  in mm
  Vector3f Tg_tran = centroid - h_def * ghat;
  Tg_tran.z() = h_def;                   //Keep body at constant height for now

  // Step 4: Default foot directions (unit circle footprint)
  Vector3f gpdef[6];
  //double angles_deg[6] = {30, 90, 150, 210, 270, 330};  //from x-axis rot to y-axis
  double angles_deg[6] = {0, -60, -120, 180, 120, 60};   //dependant on how you count the leg order. Go with Sheldon and Lotriet definition
  for (int i = 0; i < 6; ++i) {
    double angle_rad = angles_deg[i] * (M_PI / 180.0);
    gpdef[i] = Vector3f(cos(angle_rad), sin(angle_rad), -h_def);
  }

  // Step 5: Estimate average yaw from leg layout
  double phi_sum = 0.0;
  for (int i = 0; i < 6; ++i) {
    Vector2f ref_2d(gpdef[i].x(), gpdef[i].y());
    Vector2f cur_2d = (foot_tips[i] - Tg_tran).head<2>();  // XY projection

    double dot = ref_2d.dot(cur_2d);
    double cross = ref_2d.x() * cur_2d.y() - ref_2d.y() * cur_2d.x();  // 2D cross (scalar)
    phi_sum += atan2(cross, dot);
  }
  double phi = phi_sum / 6.0;

  // Step 6: Build Z-axis rotation matrix
  Matrix3f Rz;
  Rz << cos(phi), -sin(phi), 0,
        sin(phi),  cos(phi), 0,
        0,         0,        1;

  // Step 7: Assemble full 4x4 transform
  Matrix4f Tg = Matrix4f::Identity();
  Tg.block<3,3>(0,0) = Rz;
  Tg.block<3,1>(0,3) = Tg_tran;

  return Tg;
}

//convert coordinates from world frame to body frame for 1 point
//maybe rename to make this a general world to body transform function
Vector3f HantrClass::world_to_body_kins(Vector3f foot_tips_world, Matrix4f body_pose) {
  Matrix4f body_pose_inv = body_pose.inverse();  //somewhat inefficient to do repeat it.

  // Convert the 3D foot position in world frame to homogeneous coordinates
  Vector4f foot_world_homogeneous(
    foot_tips_world.x(), foot_tips_world.y(), foot_tips_world.z(), 1.0f
  );

  // Transform to body frame
  Vector4f foot_body_homogeneous = body_pose_inv * foot_world_homogeneous;

  // Store the result as a Vector3f
  Vector3f foot_tips_body = foot_body_homogeneous.head<3>();  //ref first 3 elements of the vector4f
  return foot_tips_body;
}

void HantrClass::offsetOrCopyFeet(Vector3f dest[6], const Vector3f src[6], const Vector2f offset) {
  for (int i = 0; i < 6; ++i) {
    dest[i] = src[i];
    dest[i].x() += offset.x();
    dest[i].y() += offset.y();
  }
}

HantrClass Hantr;

