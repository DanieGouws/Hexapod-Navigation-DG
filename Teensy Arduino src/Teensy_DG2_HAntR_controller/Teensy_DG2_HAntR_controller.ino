#include "InverseKin.h"
// #include "ForwKin.h"

#include <ArduinoEigen.h>
using namespace Eigen;

// Dynamixel
#include "MyDynamixel.h"
#define DXL_SERIAL Serial5
const uint8_t DEPin = 19; // DYNAMIXEL DIR PIN
MyDynamixel dxl(DXL_SERIAL, 1000000, DEPin);

// Square walking demo
const float STEP_SIZE = 50.0f;     // 10 cm per step
const float SQUARE_SIZE = 500.0f;   // 0.5 meter square
const int NUM_STEPS = 4 * (int)(SQUARE_SIZE / STEP_SIZE);  // 40 steps
int step_index = 1;  // Increment this each time you step

// Loop timers
static unsigned int stepTimerStart;     // start step every 1 s
static unsigned int stepTimerClock;
static unsigned int servoTimerStart;    // command servos every 10 ms
static unsigned int servoTimerClock;  

// foot group control
const int legs_per_group = 3;
const int group1[] = {0, 2, 4};
const int group2[] = {1, 3, 5};
int activeGroup = 0;          //0 for stationary, 1 for group 1, 2 for group 2

Vector3f foot_tips_world[6];     // Current working position of all feet, Sheldon's leg order and body frame definition
Vector3f foot_tips_world_buffer[6];  // Incoming serial data is written to this.

Vector3f feet_neutral[6] = {     //Starting and safe position of feet in world coords
  Vector3f(283.71,   0.0,     0),
  Vector3f(141.855,  -245.7,  0),
  Vector3f(-141.855, -245.7,  0),
  Vector3f(-283.71,  0.0,     0),
  Vector3f(-141.855, 245.7,   0),
  Vector3f(141.855,  245.7,   0)
};

Vector3f gravity(0, 0, -1);  // Body offset direction. From IMU, if available

Vector3f foot_tips_body[6];

// Generate the 6th order polynomials the leg paths require
class Poly6Interpolator3D {
  public:
    Poly6Interpolator3D() {}

    // Initialize with start, mid, end points and total duration
    void set(Vector3f p0, Vector3f pmid, Vector3f p1, float tf = 1.0) {
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
    Vector3f get(float t) const {
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

  private:
    float a[3][7];  // Coefficients for x, y, z
    float tf_;      // Total time
};

Poly6Interpolator3D swingPaths[6];     //store the paths for the foot tips.

InverseKinematics InKin;

//Joint Angle Variables
float theta1[6];
float theta2[6];
float theta3[6];

void setup(){
  // end_point_xyz = ... mockup some end points for group 1 and group 2 and try to get them to work. Hopefully it should walk in place when they are reached.

  Serial.begin(115200);
  Serial.println("Testing HAntR controller prototype...");    //See Paper 10.1109/ACCESS.2021.3053492

  Matrix4f body_pose = computeBodyPose(feet_neutral, gravity);   

  for(int i = 0; i < 6; i++){
    foot_tips_body[i] = world_to_body_kins(feet_neutral[i], body_pose);
    InKin.IK(&theta1[i],&theta2[i],&theta3[i],foot_tips_body[i].x(),foot_tips_body[i].y(), foot_tips_body[i].z(),i,0,0,0,0,0);   //xyz are in world coords with leg 0 at 0 angle.
  }
  SetAngles(theta1,theta2,theta3 ,10,10,10);

  //Some startup sequence to ensure the loop can run safely without start checks
  stepTimerStart = millis();     // start timer clocks
  servoTimerStart = millis();   
  activeGroup = 2;               // It will switch to group 1
  copyFeet(foot_tips_world, feet_neutral);  //Set start pos to safe pos. 
  delay(5000);                   // Ensure step timer has expired so that the step timer code is called.
}

void loop(){
  stepTimerClock = millis() - stepTimerStart;
  servoTimerClock = millis() - servoTimerStart;

  //end pointxyz of active group = read_serial_input();
  //maybe return body pose and position of all feet.
  //somehow have the ros nodes generate target points based on high level commands.
  // this is saved in a buffer which is only checked on each step change.
  copyFeet(foot_tips_world_buffer,feet_neutral);                //Mock up of target feet positions read from serial port


  // if (all tips have touched down (do with expired timer for now)){
  //  switch active leg group
  //  calculate coefficients for next step leg moves.
  //  start 1s step timer
  // }
  //Each time the feet reach the ground, the foot target buffer is used to create new foot paths
  if(stepTimerClock > 3000){    // The 1s Step loop
    stepTimerStart = millis();

    Vector2f square_offset = getSquareStepPoint(step_index);               // Demo Square
    offsetFeet(foot_tips_world_buffer, feet_neutral, square_offset);
    step_index = (step_index + 1);// % NUM_STEPS;
    
    if(activeGroup == 1){       // switch active feet group
      activeGroup = 2;
    }
    else if(activeGroup == 2){
      activeGroup = 1;
    }

    for(int i = 0; i < 6; i++){
      Vector3f pmid = (foot_tips_world[i] + foot_tips_world_buffer[i]) * 0.5;   // midpoint between p0 and p1
      pmid.z() += 50.0;              // height to make foot rise above ground
      swingPaths[i].set(foot_tips_world[i], pmid, foot_tips_world_buffer[i], 3.0);   // calculate poly6 to create path for foot to follow, not necessary for all 6
    }

  Serial.print("Switched to Group " + String(activeGroup));
  }

  // Assume that feet reached previous commanded points with some error if they hit ground. 
  // Sample 6 polynomials and get next required foot positions.
  // Calculate required body pose. Transform foot positions to body frame.
  // Kins to servo Angles. Write to servos.
  if(servoTimerClock >= 10){                  //100 Hz Servo write loop
    servoTimerStart = millis();

    for(int i=0,j = 0; i < legs_per_group; i++){    // find indexes of active feet. Improve maybe.
      if(activeGroup == 1){
        j = group1[i];
      }
      else if(activeGroup == 2){
        j = group2[i];
      }
      foot_tips_world[j] = swingPaths[j].get(stepTimerClock/1000.0);   // Sample polynomial for foot pos.
    }
      
    Matrix4f body_pose = computeBodyPose(foot_tips_world, gravity);

    for(int i = 0; i < 6; i++){
      foot_tips_body[i] = world_to_body_kins(foot_tips_world[i], body_pose);                 // transform point from world to body frame
      InKin.IK(&theta1[i],&theta2[i],&theta3[i],foot_tips_body[i].x(),foot_tips_body[i].y(), foot_tips_body[i].z(),i,0,0,0,0,0);   //xyz are in world coords with leg 0 at 0 angle.
    }
    SetAngles(theta1,theta2,theta3 ,10,10,10);   // Write to servos
  }

}

// Function to read servo angle arrays (1 element for each leg)  in rads 
// and speeds (just one per joint type) in rpm to drive the servo at. 
// Syncmove is one instruction and quite fast.
void SetAngles(float* th1,float* th2,float* th3 ,float spd1,float spd2, float spd3){

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
  index = index % NUM_STEPS;  // Wrap around

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

