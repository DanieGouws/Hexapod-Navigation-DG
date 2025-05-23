#include "HAntR_functions.h"



//Get Body pos and orientation as a 4x4 mat.
// Body is const distance above feet centroid, pitch and roll level, yaw aimed in feet direction.
Matrix4f computeBodyPose(Vector3f foot_tips[6], Vector3f gravity) {
  
  // Step 1: Compute centroid of foot tips
  Vector3f centroid(0, 0, 0);
  for (int i = 0; i < 6; ++i) {
    centroid += foot_tips[i];
  }
  centroid /= 6.0;

  // Step 2: Normalize gravity vector
  Vector3f ghat = gravity.normalized();

  // Step 3: Compute body translation
  const float h_def = 150.0;  // Default body height above feet centroid  in mm
  Vector3f Tg_tran = centroid - h_def * ghat;
  Tg_tran.z() = h_def;                   //Keep body at constant height for now

  // Step 4: Default foot directions (unit circle footprint)
  Vector3f gpdef[6];
  //float angles_deg[6] = {30, 90, 150, 210, 270, 330};  //from x-axis rot to y-axis
  float angles_deg[6] = {0, -60, -120, 180, 120, 60};   //dependant on how you count the leg order. Go with Sheldon and Lotriet definition
  for (int i = 0; i < 6; ++i) {
    float angle_rad = angles_deg[i] * (M_PI / 180.0);
    gpdef[i] = Vector3f(cos(angle_rad), sin(angle_rad), -h_def);
  }

  // Step 5: Estimate average yaw from leg layout
  float phi_sum = 0.0;
  for (int i = 0; i < 6; ++i) {
    Vector2f ref_2d(gpdef[i].x(), gpdef[i].y());
    Vector2f cur_2d = (foot_tips[i] - Tg_tran).head<2>();  // XY projection

    float dot = ref_2d.dot(cur_2d);
    float cross = ref_2d.x() * cur_2d.y() - ref_2d.y() * cur_2d.x();  // 2D cross (scalar)
    phi_sum += atan2(cross, dot);
  }
  float phi = phi_sum / 6.0;

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

//convert coordinates from world frame to body frame
//maybe rename to make this a general world to body transform function
Vector3f world_to_body_kins(Vector3f foot_tips_world, Matrix4f body_pose) {
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

void copyFeet(Vector3f dest[6], Vector3f src[6]) {
  for (int i = 0; i < 6; ++i) dest[i] = src[i];
}



// Only for creating a square walking demo.
// Returns the (x, y) coordinate on the square for step_index
Vector2f getSquareStepPoint(int index) {
  // index = index % NUM_STEPS;  // Wrap around

  float total_len = STEP_SIZE * index;

  if (total_len < SQUARE_SIZE) {
    return Vector2f(total_len, 0);  // Bottom edge (right)
  } else if (total_len < 2 * SQUARE_SIZE) {
    return Vector2f(SQUARE_SIZE, total_len - SQUARE_SIZE);  // Right edge (up)
  } else if (total_len < 3 * SQUARE_SIZE) {
    return Vector2f(SQUARE_SIZE - (total_len - 2 * SQUARE_SIZE), SQUARE_SIZE);  // Top edge (left)
  } else if (total_len < 4 * SQUARE_SIZE){
    return Vector2f(0, SQUARE_SIZE - (total_len - 3 * SQUARE_SIZE));  // Left edge (down)
  } else if(total_len < 4.2 * SQUARE_SIZE){
    return Vector2f(0,0);
  } else {
    while(1);
  }
}

void offsetFeet(Vector3f dest[6], const Vector3f src[6], const Vector2f& offset) {
  for (int i = 0; i < 6; ++i) {
    dest[i] = src[i];
    dest[i].x() += offset.x();
    dest[i].y() += offset.y();
  }
}

Poly6Interpolator3D::Poly6Interpolator3D(){}

// Initialize with start, mid, end points and total duration
void Poly6Interpolator3D::set(Vector3f p0, Vector3f pmid, Vector3f p1, float tf = 1.0) {
  tf_ = tf;

  for (int i = 0; i < 3; ++i) {
    float s0 = p0[i];
    float sm = pmid[i];
    float s1 = p1[i];

    float dx1 = sm - s0;
    float dx2 = s1 - s0;

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
Vector3f Poly6Interpolator3D::get(float t) const {
  t = constrain(t, 0.0f, tf_);
  Vector3f p;
  for (int i = 0; i < 3; ++i) {
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    float t5 = t4 * t;
    float t6 = t5 * t;
    p[i] = a[i][0] + a[i][1]*t + a[i][2]*t2 + a[i][3]*t3 +
            a[i][4]*t4 + a[i][5]*t5 + a[i][6]*t6;
  }
  return p;
}



// // Generate the 6th order polynomials the leg paths require
// class Poly6Interpolator3D {
//   public:
//     Poly6Interpolator3D() {}

    

    

//   private:
//     float a[3][7];  // Coefficients for x, y, z
//     float tf_;      // Total time
// };