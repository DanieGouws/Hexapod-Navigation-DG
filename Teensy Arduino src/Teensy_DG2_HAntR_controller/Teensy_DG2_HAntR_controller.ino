#include "InverseKin.h"
#include "ForwKin.h"

#include <ArduinoEigen.h>
using namespace Eigen;

const float h_def = 150.0;  // Default body height above ground in mm

// Dynamixel
#include "MyDynamixel.h"
#define DXL_SERIAL Serial5
const uint8_t DEPin = 19; // DYNAMIXEL DIR PIN
MyDynamixel dxl(DXL_SERIAL, 1000000, DEPin);

// Loop timers
static unsigned int stepTimerStart;     // start step every 1 s
static unsigned int servoTimerStart;    // command servos every 10 ms


// foot tip (end of step) target points in world frame
float xfoot_target[6];
float yfoot_target[6];
float zfoot_target[6];

// foot group control
const int legs_per_group = 3;
const int group1[] = {0, 2, 4};
const int group2[] = {1, 3, 5};
int activeGroup = 0;          //0 for stationary, 1 for group 1, 2 for group 2

//buffer to save foot pos in world and body
float xfoot_world[6];
float yfoot_world[6];
float zfoot_world[6];

float xfoot_body[6];
float yfoot_body[6];
float zfoot_body[6];

InverseKinematics InKin;

//Joint Angle Variables
float theta1[6];
float theta2[6];
float theta3[6];

Vector3f foot_tips_world1[6] = {     //Sheldon's leg order and body frame definition
  Vector3f(283.71,   0.0,     0),
  Vector3f(141.855,  -245.7,  0),
  Vector3f(-141.855, -245.7,  0),
  Vector3f(-283.71,  0.0,     0),
  Vector3f(-141.855, 245.7,   0),
  Vector3f(141.855,  245.7,   0)
};

Vector3f foot_tips_world2[6] = {     //declare a second set of points to mock up periodic movement and timers
  Vector3f(283.71,   0.0,     50),
  Vector3f(141.855,  -245.7,  0),
  Vector3f(-141.855, -245.7,  50),
  Vector3f(-283.71,  0.0,     0),
  Vector3f(-141.855, 245.7,   50),
  Vector3f(141.855,  245.7,   0)
};

Vector3f foot_tips_world[6];
Vector3f foot_tips_body[6];


void setup(){

  

  // end_point_xyz = ... mockup some end points for group 1 and group 2 and try to get them to work. Hopefully it should walk in place when they are reached.

  Serial.begin(115200);
  Serial.println("Testing HAntR controller prototype...");

  

    Vector3f gravity(0, 0, -1);  // From IMU, if available
    for (int i = 0; i < 6; ++i) {
      foot_tips_world[i] = foot_tips_world1[i];
    }
    Matrix4f body_pose = computeBodyPose(foot_tips_world, gravity);

    // Serial.println("Body Pose Matrix (Tg):");
    // for (int i = 0; i < 4; i++) {
    //   for (int j = 0; j < 4; j++) {
    //     Serial.print(body_pose(i, j), 4);
    //     Serial.print("\t");
    //   }
    //   Serial.println();
    // }

  // Serial.println();
  for(int i = 0; i < 6; i++){
    //world_to_body_kins(&xfoot_body[i], &yfoot_body[i], &zfoot_body[i], xfoot_world[i], yfoot_world[i],zfoot_world[i], body_pose);
    foot_tips_body[i] = world_to_body_kins(foot_tips_world[i], body_pose);
    InKin.IK(&theta1[i],&theta2[i],&theta3[i],foot_tips_body[i].x(),foot_tips_body[i].y(), foot_tips_body[i].z(),i,0,0,0,0,0);   //xyz are in world coords with leg 0 at 0 angle.
   
    // Serial.print(foot_tips_body[i].x());
    // Serial.print(" ");
    // Serial.print(foot_tips_body[i].y());
    // Serial.print(" ");
    // Serial.println(foot_tips_body[i].z());
  }
  // Serial.println();
  SetAngles(theta1,theta2,theta3 ,10,10,10);
  delay(3000);

  //some startup sequence to make it go to safe mode
  servoTimerStart = millis();   // start timer clocks
  stepTimerStart = millis();
  activeGroup = 1; 
}

void loop(){
  

  //end pointxyz of active group = read_serial_input();
  //return body pose and position of all feet.
  //somehow have the ros nodes generate target points based on high level commands.


  // if (all tips have touched down (do with expired timer for now)){
  //  switch active leg group
  //  calculate coefficients for next step leg moves.
  //  start 1s step timer
      
  // }

  //the above
  if(millis() - stepTimerStart > 1000){    // The 1s Step loop
    
    if(activeGroup == 1)activeGroup = 2;
    else if(activeGroup == 2)activeGroup = 1;

    //Poly[active legs].calculate_poly_coefs(start point xyz, end pointxyz);   //here the targets are converted to an executable buffer sortof. End points need to be set before this happens.
    stepTimerStart = millis();
  }

  if(millis() - servoTimerStart >= 10){                  //100 Hz loop
    servoTimerStart = millis();

    //assume that legs reached previous commanded points with some error. 
    //based on step timer, sample 9 polynomials and get next required foot pos.
    

    // need foot tip coords in world frame. Assume we have those.

    
    // At each time step, get foot tip coords in world frame to command servos.
    // Stationary world coords stay unchanged
    // for(int i = 0; i < legs_per_group; i++){
    //   if(activeGroup == 1){
    //     j = group1[i];
    //   }
    //   else if(activeGroup == 2){
    //     j = group2[i]
    //   }
      
      
    //   //xfoot_world[j] = sample_line(j)           //probably need a class to track the coefs of each leg.
    //   // yfoot_world[j] = sample_line(j)
    //   // zfoot_world[j] = sample_6o_polynomial(j)  //non-mentioned foot tips should just stay stationary in world coords
    // }
    if(activeGroup == 1){                     //choose legs depending on our step timer. Mockup for now.
      for (int i = 0; i < 6; ++i) {
        foot_tips_world[i] = foot_tips_world1[i];
      }
    }else if(activeGroup == 2){
      for (int i = 0; i < 6; ++i) {
        foot_tips_world[i] = foot_tips_world2[i];
      }
    }

    Vector3f gravity(0, 0, -1);  // From IMU, if available
    Matrix4f body_pose = computeBodyPose(foot_tips_world, gravity);

    for(int i = 0; i < 6; i++){
    foot_tips_body[i] = world_to_body_kins(foot_tips_world[i], body_pose);
    InKin.IK(&theta1[i],&theta2[i],&theta3[i],foot_tips_body[i].x(),foot_tips_body[i].y(), foot_tips_body[i].z(),i,0,0,0,0,0);   //xyz are in world coords with leg 0 at 0 angle.
  }

  SetAngles(theta1,theta2,theta3 ,10,10,10);
  }

}

// void world_to_body_kins(float *xfoot_body, float *yfoot_body,float *zfoot_body, float xfoot_world, float yfoot_world, float zfoot_world, int body_pose){
//   //kins to go from foot tip in the world space to foot tip xyz in body space

//   //placeholder
//   *xfoot_body = xfoot_world;
//   *yfoot_body = yfoot_world;
//   *zfoot_body = zfoot_world;
//   body_pose;


// }

void SetAngles(float* th1,float* th2,float* th3 ,float spd1,float spd2, float spd3)
{
  // Function to read servo angle arrays (1 element for each leg)  in rads 
  // and speeds (just one per motor type) in rpm to drive the servo at. 
  // Syncmove is one instruction and quite fast.

  double Angle[18];
  double Spd[18];
  uint8_t Id[18];

  for(int i = 0; i < 6; i++){
        Angle[3*i  ] =  th1[i];     //package values correctly. 6 legs, 3 servos/leg. Signs for proper kins.
        Angle[3*i+1] = -th2[i];
        Angle[3*i+2] = -th3[i];

        Spd[3*i  ] = spd1;
        Spd[3*i+1] = spd2;
        Spd[3*i+2] = spd3;
      }
      for(uint8_t i = 0; i < 18; i++){Id[i] = i;}   //Generate Servo ID array {0,1,..., 17}


  dxl.SyncMove(Id,Angle,Spd,18);    // Drive the servos
}

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
  Vector3f Tg_tran = centroid - h_def * ghat;

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

//Tested with the below:
/*
Vector3f foot_tips[6] = {
    Vector3f(86.6, 50, 0),
    Vector3f(0, 100, 0),
    Vector3f(-86.6, 50, 0),
    Vector3f(-86.6, -50, 0),
    Vector3f(0, -100, 0),
    Vector3f(86.6, -50, 0)
  };

  Vector3f gravity(0, 0, -1);  // From IMU, if available

  Matrix4f Tg = computeBodyPose(foot_tips, gravity);

  Serial.println("Body Pose Matrix (Tg):");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Serial.print(Tg(i, j), 4);
      Serial.print("\t");
    }
    Serial.println();
  }
*/

//convert coordinates from world frame to body frame, seems to work
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

class Interpolator3D {       //
  public:
    Interpolator3D() {}

    Interpolator3D(const Vector3f& start_point, const Vector3f& end_point) {
      setPoints(start_point, end_point);
    }

    void setPoints(const Vector3f& start_point, const Vector3f& end_point) {
      p0 = start_point;
      direction = end_point - start_point;
    }

    // Returns interpolated point for t in [0, 1]
    Vector3f interpolate(float t) const {
      t = constrain(t, 0.0f, 1.0f);
      return p0 + t * direction;
    }

  private:
    Vector3f p0;
    Vector3f direction;
};






