#include "Kinematics.h"
#include "HAntR_functions.h"
#include "Poly6Interpolator3D.h"
#include "Misc_demo_functions.h"
#include "Mujoco_HIL_sim.h"

#include <ArduinoEigen.h>
using namespace Eigen;


// Dynamixel Interface
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
#define STEP_PERIOD_ms 500.0
#define SERVO_PERIOD_ms 80.0

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
Poly6Interpolator3D swingPaths[6];     //store the paths for the foot tips.

//Joint Angle Variables
double angles[18];

//Some waypoints for testing Waypoint nav?
Vector2f waypoints[7] = {     //Some waypoints (x,y) in m to play with
  Vector2f(0.5, 1),  
  Vector2f(1,-2),
  Vector2f(2, 0.5),
  Vector2f(1.2, 1),
  Vector2f(0.5, 2),
  Vector2f(0, 1.5)
};

Vector2f hex_pos = Vector2f(0,0);  // Position of the hexapod in m in the world frame.
double hex_heading = 0.0;             // Heading of the hexapod [rad] in the world frame.
#define vel_forward 0.1      //m/s
double turn_rate = M_PI/10;          //rad/s


void setup(){
  // end_point_xyz = ... mockup some end points for group 1 and group 2 and try to get them to work. Hopefully it should walk in place when they are reached.

  Serial.begin(115200);
  while(!Serial);
  delay(500);
  Serial.println("Testing HAntR controller prototype...");    //See Paper 10.1109/ACCESS.2021.3053492

  Matrix4f body_pose = Hantr.computeBodyPose(feet_neutral, gravity);   

  for(int i = 0; i < 6; i++){
    foot_tips_body[i] = Hantr.world_to_body_kins(feet_neutral[i], body_pose);
    Kins.body_to_joint(&angles[3*i],&angles[3*i+1],&angles[3*i+2], foot_tips_body[i].x(),foot_tips_body[i].y(), foot_tips_body[i].z(), i);
    
    angles[3*i+1] = -angles[3*i+1];  //sign between kins output and servo/sim
    angles[3*i+2] = -angles[3*i+2];
  }
  writeServoHardware(angles ,10.0);
  Mujoco.writeServoSim(angles, 10.0);

  //Some startup sequence to ensure the loop can run safely without start checks
  stepTimerStart = millis();     // start timer clocks
  servoTimerStart = millis();   
  activeGroup = 2;               // It will switch to group 1
  Hantr.offsetOrCopyFeet(foot_tips_world, feet_neutral); //Set start pos to safe pos. 
  delay(5000);                   // Ensure step timer has expired so that the step timer code is called.
}

void loop(){
  stepTimerClock = millis() - stepTimerStart;
  servoTimerClock = millis() - servoTimerStart;

  //end pointxyz of active group = read_serial_input();
  //maybe return body pose and position of all feet.
  //somehow have the ros nodes generate target points based on high level commands.
  // this is saved in a buffer which is only checked on each step change.
  // copyFeet(foot_tips_world_buffer,feet_neutral);                //Mock up of target feet positions read from serial port


  // if (all tips have touched down (do with expired timer for now)){
  //  switch active leg group
  //  calculate coefficients for next step leg moves.
  //  start 1s step timer
  // }
  //Each time the feet reach the ground, the foot target buffer is used to create new foot paths
  if(stepTimerClock > STEP_PERIOD_ms){    // The 1s Step loop
    stepTimerStart = millis();

    // Vector2f square_offset = Demo.getSquareStepPoint(step_index);               // Demo Square

    //
    static int wpt_index=1;
    double Kly = 0.5;  //characteristic decay length[m]
    double Kturn = 5.0; //number of seconds to remove the error between target heading and current heading at initial turn rate.
    


    Vector2f wpt_src = waypoints[(wpt_index-1)%7];
    Vector2f wpt_dest = waypoints[wpt_index];

    Serial.print("Current dest:");
    Serial.print(wpt_dest.x());
    Serial.print(" ");
    Serial.print(wpt_dest.y());
    Serial.print("  Current hexapod position:");
    Serial.print(hex_pos.x());
    Serial.print(" ");
    Serial.println(hex_pos.y());
    

    if((waypoints[wpt_index]-hex_pos).norm() < 0.1){
      Serial.print("Found waypoint ");
      Serial.println(wpt_index);
      wpt_index = (wpt_index+1)%7;              //switch to next one if close enough
    }
      

    double psi_track = atan2(wpt_dest.y()-wpt_src.y(), wpt_dest.x()-wpt_src.x());

    // Matrix2f track_rot = Matrix2f(cos(psi_track), sin(psi_track), -sin(psi_track), cos(psi_track));
    // Vector2f track_offset = hex_pos - wpt_src;
    // Vector2f track_answer = track_rot * track_offset;

    // double x_inTrack =  track_answer.x(); //change his angle def. Calculate with elegant frame matrices.
    // double y_crossTrack = track_answer.y();

    // double x_inTrack =     cos(psi_track)*(hex_pos.x()- wpt_src.x()) + sin(psi_track)*(hex_pos.y()- wpt_src.y()); //change his angle def. Calculate with elegant frame matrices.
    double y_crossTrack = -sin(psi_track)*(hex_pos.x()- wpt_src.x()) + cos(psi_track)*(hex_pos.y()- wpt_src.y());
    y_crossTrack *= -1;

    double psi_ref = psi_track + atan(y_crossTrack/Kly);  //Kl is like a sort of half distance in x as it decays to the track.
    //not atan(y/x).... It just goes in a straight line to target.

    turn_rate = (psi_ref- hex_heading)/Kturn;

    hex_heading += turn_rate*STEP_PERIOD_ms/1000.0;
    Serial.print("hex_heading: ");
    Serial.println(hex_heading);

    //v = dx/dt
    //dx = v*dt
    
    double step_length =  vel_forward*STEP_PERIOD_ms/1000.0;  //m/s * s = m
    hex_pos.x() += step_length*cos(hex_heading);
    hex_pos.y() += step_length*sin(hex_heading);

    //use only mm after this point
    
    Hantr.offsetOrCopyFeet(foot_tips_world_buffer, feet_neutral, hex_pos*1000.0, hex_heading);
    // step_index = (step_index + 1);// % NUM_STEPS;
    
    if(activeGroup == 1){       // switch active feet group
      activeGroup = 2;
    }
    else if(activeGroup == 2){
      activeGroup = 1;
    }

    for(int i = 0; i < 6; i++){
      Vector3f pmid = (foot_tips_world[i] + foot_tips_world_buffer[i]) * 0.5;   // midpoint between p0 and p1
      pmid.z() += 50.0;              // height to make foot rise above ground
      swingPaths[i].set(foot_tips_world[i], pmid, foot_tips_world_buffer[i]);   // calculate poly6 to create path for foot to follow, not necessary for all 6
    }

  Serial.println("Switched to Group " + String(activeGroup));
  Serial.println();
  }

  // Assume that feet reached previous commanded points with some error if they hit ground. 
  // Sample 6 polynomials and get next required foot positions.
  // Calculate required body pose. Transform foot positions to body frame.
  // Kins to servo Angles. Write to servos.
  if(servoTimerClock >= SERVO_PERIOD_ms){                  //100 Hz Servo write loop
    servoTimerStart = millis();

    for(int i=0,j = 0; i < legs_per_group; i++){    // find indexes of active feet. Improve maybe.
      if(activeGroup == 1){
        j = group1[i];
      }
      else if(activeGroup == 2){
        j = group2[i];
      }
      foot_tips_world[j] = swingPaths[j].get(stepTimerClock/STEP_PERIOD_ms);   // Sample polynomial for foot pos.
    }
      
    Matrix4f body_pose = Hantr.computeBodyPose(foot_tips_world, gravity);

    for(int i = 0; i < 6; i++){
      foot_tips_body[i] = Hantr.world_to_body_kins(foot_tips_world[i], body_pose);                 // transform point from world to body frame
      Kins.body_to_joint(&angles[0+3*i],&angles[1+3*i],&angles[2+3*i], foot_tips_body[i].x(),foot_tips_body[i].y(), foot_tips_body[i].z(), i);
          
      angles[1+3*i] = -angles[1+3*i];  //sign between kins output and servo/sim
      angles[2+3*i] = -angles[2+3*i];
    }

    writeServoHardware(angles ,10.0);
    Mujoco.writeServoSim(angles, 10.0);
  }

}

// Function to read servo angle arrays (1 element for each leg)  in rads 
// and speeds (just one per joint type) in rpm to drive the servo at. 
// Syncmove is one instruction and quite fast.
void writeServoHardware(double angles[18],double spd){
  double Spd[18];
  uint8_t Id[18];

  for(int i = 0; i < 18; i++){
    Spd[i] = spd;
    Id[i] = i;
  }
  dxl.SyncMove(Id,angles,Spd,18);    // Drive the servos
}