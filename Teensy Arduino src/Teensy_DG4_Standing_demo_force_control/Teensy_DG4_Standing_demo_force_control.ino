#include "InverseKin.h"
#include "ForwKin.h"
#include "HAntR_functions.h"

#include <ArduinoEigen.h>
using namespace Eigen;



// Dynamixel
#include "MyDynamixel.h"
#define DXL_SERIAL Serial5
const uint8_t DEPin = 19; // DYNAMIXEL DIR PIN
MyDynamixel dxl(DXL_SERIAL, 1000000, DEPin);

// Square walking demo
int step_index = 1;  // Increment this each time you step

// Loop timers
static unsigned int stepTimerStart;     // start step every 1 s
static unsigned int stepTimerClock;
static unsigned int servoTimerStart;    // command servos every 10 ms
static unsigned int servoTimerClock; 
#define STEP_PERIOD_ms 1000.0
#define SERVO_PERIOD_ms 50.0

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
Vector3f foot_tips_body_read[6];



Poly6Interpolator3D swingPaths[6];     //store the paths for the foot tips.

InverseKinematics InKin;

//Joint Angle Variables
float theta1[6];
float theta2[6];
float theta3[6];

void setup(){
  // end_point_xyz = ... mockup some end points for group 1 and group 2 and try to get them to work. Hopefully it should walk in place when they are reached.

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Testing HAntR controller prototype...");    //See Paper 10.1109/ACCESS.2021.3053492

  Matrix4f body_pose = computeBodyPose(feet_neutral, gravity);   

  for(int i = 0; i < 6; i++){
    foot_tips_body[i] = world_to_body_kins(feet_neutral[i], body_pose);
    InKin.IK(&theta1[i],&theta2[i],&theta3[i],foot_tips_body[i].x(),foot_tips_body[i].y(), foot_tips_body[i].z(),i,0,0,0,0,0);   //xyz are in world coords with leg 0 at 0 angle.
  }
  SetAngles(theta1,theta2,theta3 ,10,10,10);
  SendServoStates(theta1,theta2,theta3 ,10,10,10);     //for Mujoco HIL integration

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
  // static unsigned int ReadStart = 0;

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
  // if(stepTimerClock > STEP_PERIOD_ms){    // The 1s Step loop
  //   stepTimerStart = millis();

  //   Vector2f square_offset = getSquareStepPoint(step_index);               // Demo Square
  //   offsetFeet(foot_tips_world_buffer, feet_neutral, square_offset);
  //   step_index = (step_index + 1);// % NUM_STEPS;
    
  //   if(activeGroup == 1){       // switch active feet group
  //     activeGroup = 2;
  //   }
  //   else if(activeGroup == 2){
  //     activeGroup = 1;
  //   }

  //   for(int i = 0; i < 6; i++){
  //     Vector3f pmid = (foot_tips_world[i] + foot_tips_world_buffer[i]) * 0.5;   // midpoint between p0 and p1
  //     pmid.z() += 50.0;              // height to make foot rise above ground
  //     swingPaths[i].set(foot_tips_world[i], pmid, foot_tips_world_buffer[i], STEP_PERIOD_ms/1000.0);   // calculate poly6 to create path for foot to follow, not necessary for all 6
  //   }

  // Serial.print("Switched to Group " + String(activeGroup));
  // }

  // Assume that feet reached previous commanded points with some error if they hit ground. 
  // Sample 6 polynomials and get next required foot positions.
  // Calculate required body pose. Transform foot positions to body frame.
  // Kins to servo Angles. Write to servos.
  if(servoTimerClock >= SERVO_PERIOD_ms){                  //100 Hz Servo write loop
    servoTimerStart = millis();
    


         //read from simulation
    /*
    double angles[18];
    if (ReadAllServoAngles(angles)) {    //if the pc is too busy, just ignore it this loop
      for (int leg = 0; leg < 6; leg++) {
        float yaw    = angles[leg * 3 + 0];
        float pitch1 = -angles[leg * 3 + 1];
        float pitch2 = -angles[leg * 3 + 2];
        float Px, Py, Pz;
        FK03_inbody(Px, Py, Pz, yaw, pitch1, pitch2, leg);
        foot_tips_body_read[leg] = Vector3f(Px, Py, Pz);
      }
    }*/
    
      for (int leg = 0; leg < 6; leg++) {               //read from real hexapod
        float yaw    = dxl.PresentPos(leg * 3 + 0);
        float pitch1 = -dxl.PresentPos(leg * 3 + 1);
        float pitch2 = -dxl.PresentPos(leg * 3 + 2);
        float Px, Py, Pz;
        FK03_inbody(Px, Py, Pz, yaw, pitch1, pitch2, leg);
        foot_tips_body_read[leg] = Vector3f(Px, Py, Pz);
      }
    Matrix4f body_pose = computeBodyPose(foot_tips_world, gravity);

    Vector3f error[6];
    static double z_offset[6] =  {0.0};
    for(int i = 0; i < 6; i++){
      foot_tips_body[i] = world_to_body_kins(foot_tips_world[i], body_pose);                 // transform point from world to body frame
      //do some filter stuff
      error[i] = foot_tips_body_read[i] - (foot_tips_body[i] + Vector3f(0,0,z_offset[i]));  //choose some appropriate gain given loop speed. Vector from command to read point.
    }

    Vector3f error_ave(0,0,0);
    for (int i = 0; i < 6; ++i) {
      error_ave += error[i];
    }
    error_ave /= 6.0;
    Serial.print("GRF: ");
    Serial.print(error_ave.x());
    Serial.print("  ");
    Serial.print(error_ave.y());
    Serial.print("  ");
    Serial.println(error_ave.z());

   
    for(int i = 0; i < 6; i++){
      if(abs((error[i]- error_ave).z()) > 0.2){
        z_offset[i] += (error[i]- error_ave).z()*20.0*SERVO_PERIOD_ms/1000.0;     //I-controller for foot height  //5 for sim
      }
      InKin.IK(&theta1[i],&theta2[i],&theta3[i],foot_tips_body[i].x(),foot_tips_body[i].y(), foot_tips_body[i].z() + z_offset[i],i,0,0,0,0,0);   //xyz are in world coords with leg 0 at 0 angle.
      Serial.print( foot_tips_body[i].z() + z_offset[i]);
      Serial.print("  ");
      Serial.println(foot_tips_body_read[i].z());
    }
    SetAngles(theta1,theta2,theta3 ,50,50,50);   // Write to servos
    SendServoStates(theta1,theta2,theta3 ,10,10,10);

    // Serial.print("Total servo loop length us: ");
    // Serial.println(micros() - ReadStart);
    // if(micros() - ReadStart > SERVO_PERIOD_ms*1.01*1000.0){
    //   Serial.println("TIMEOUT!!!!");
    // }
    // ReadStart = micros();
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

void SendServoStates(float* th1, float* th2, float* th3, float spd1, float spd2, float spd3) {
  float angles[18];
  float speeds[18];

  // Package joint angles back into linear array
  for (int i = 0; i < 6; i++) {
    angles[3*i  ] = th1[i];
    angles[3*i+1] = -th2[i];  // Undo signs if SetAngles had sign convention
    angles[3*i+2] = -th3[i];

    speeds[3*i  ] = spd1;
    speeds[3*i+1] = spd2;
    speeds[3*i+2] = spd3;
  }

  // Optional: define a sync byte to mark start of message
  const uint8_t sync = 0xAB;
  Serial.write(sync);

  // Send angles first
  Serial.write((uint8_t*)angles, sizeof(angles));

  // Optionally send speeds
  Serial.write((uint8_t*)speeds, sizeof(speeds));

}

bool ReadAllServoAngles(double angles[18]) {
  // Flush buffer
  while (Serial.available()) Serial.read();

  // Send request
  Serial.write(0xCE);

  const int total_bytes = 18 * 8;
  uint8_t buf[total_bytes];
  int received = 0;
  unsigned long start = millis();

  // Read 144 bytes
  while (received < total_bytes) {
    if (Serial.available()) {
      buf[received++] = Serial.read();
    }
    if (millis() - start > 100) {
      Serial.println("Timeout reading all servo angles");
      return false;
    }
  }

  // Serial.print("Time to response[ms]: ");
  // Serial.println(millis() - start);

  // Convert byte buffer into doubles
  memcpy(angles, buf, total_bytes);  // byte-safe bulk copy

  return true;
}






