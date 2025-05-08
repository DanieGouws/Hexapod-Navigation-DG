#include "InverseKin.h"
#include "ForwKin.h"


// Loop timers
static unsigned int stepTimerStart;     // start step every 1 s
static unsigned int servoTimerStart;    // command servos every 10 ms


// foot tip (end of step) target points in world frame
float xfoot_target[6];
float yfoot_target[6];
float zfoot_target[6];

// foot group control
const int legs_per_group = 3;
const int group1 = {0, 2, 4};
const int group2 = {1, 3, 5};
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





/*
#include "interpo.h"


#include <ArduinoJson.h>



//Some variables



//Joint Angle Variables for SetAngle Mode
float A_theta1[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float A_theta2[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float A_theta3[6]={-1.5, -1.5, -1.5, -1.5, -1.5, -1.5};

//Joint Angle Variables for Teleop demo Mode
float T_theta1[6];
float T_theta2[6];
float T_theta3[6];


// Dynamixel
#include "MyDynamixel.h"
#define DXL_SERIAL Serial5
const uint8_t DEPin = 19; // DYNAMIXEL DIR PIN
MyDynamixel dxl(DXL_SERIAL, 1000000, DEPin);

int mode = 2;     // Control this from ROS2 somehow
int startUp = 0;

//Pitch roll input subscriber
float pitchInput = 0.0;
float rollInput = 0.0;

double tx=0,ty=0,tz=0;
float troll=0,tpitch=0,tyaw=0;

float FeetOnFloor = 0;

#define Pathsize 7
#define Steps (Pathsize * 2 - 2)
float XPath[6][Steps];
float YPath[6][Steps];
float ZPath[6][Steps];
float TurnPath[Steps];
float dt = 0;
int startSetPath = 0;
int standBegin = 0;

//Buffers to save gait path until the current step has finished.
bool newPathPending = false;
float XPathBuf[6][Steps];
float YPathBuf[6][Steps];
float ZPathBuf[6][Steps];
float TurnPathBuf[Steps];

String motor_feedback[18] = "";

*/

void setup(){

  //some startup sequence to make it go to safe mode
  servoTimerStart = millis();   // start timer clock
  stepTimerStart = millis();

  // end_point_xyz = ... mockup some end points for group 1 and group 2 and try to get them to work. Hopefully it should walk in place when they are reached.
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
  if(millis() - stepTimerStart > 1000){    // The steptimer has expired
    
    if(activeGroup == 1)activeGroup = 2;
    else if(activeGroup == 2)activeGroup = 1;

    Poly[active legs].calculate_poly_coefs(start point xyz, end pointxyz);   //here the targets are converted to an executable buffer sortof. End points need to be set before this happens.
    stepTimerStart = millis();
  }

  if(millis() - servoTimerStart >= 10){                  //100 Hz loop
    servoTimerStart = millis();

    //assume that legs reached previous commanded points with some error. 
    //based on step timer, sample 9 polynomials and get next required foot pos.
      



    // need foot tip coords in world frame. Assume we have those.

    
    // At each time step, get foot tip coords in world frame to command servos.
    // Stationary world coords stay unchanged
    for(int i = 0; i < legs_per_group; i++){
      if(activeGroup == 1){
        j = group1[i];
      }
      else if(activeGroup == 2){
        j = group2[i]
      }
      
      
      xfoot_world[j] = sample_line(j)           //probably need a class to track the coefs of each leg.
      yfoot_world[j] = sample_line(j)
      zfoot_world[j] = sample_6o_polynomial(j)  //non-mentioned foot tips should just stay stationary in world coords
    }

    //body_pose = calculate_body_pose(pos of each foot in world points);   //position and orientation of the body. needed for world to body kins
    

    // take foot tip coords in world frame, convert to body, convert to angles and drive servos.
    for(int i = 0; i < 6; i++){
      
      world_to_body_kins(&xfoot_body[i], &yfoot_body[i], &zfoot_body[i], xfoot_world[i], yfoot_world[i],zfoot_world[i], body_pose);
      InKin.IK(&theta1[i],&theta2[i],&theta3[i],xfoot_body[i],yfoot_body[i], zfoot_body[i],i,0,0,0,0,0);   //xyz are in world coords with leg 0 at 0 angle.
    }
    SetAngles(theta1,theta2,theta3 ,20,20,20)
  }

}

void world_to_body_kins(float *xfoot_body, float *yfoot_body,float *zfoot_body, float xfoot_world, float yfoot_world, float zfoot_world, int body_pose){
  //kins to go from foot tip in the world space to foot tip xyz in body space

  //placeholder
  *xfoot_body = xfoot_world;
  *yfoot_body = yfoot_world;
  *zfoot_body = zfoot_world;
  body_pose;


}

void SetAngles(float* th1,float* th2,float* th3 ,float spd1,float spd2, float spd3)
{
  // Function to read servo angle arrays (1 element for each leg)  in rads 
  // and speeds (just one per motor type) in rpm to drive the servo at. 
  // Syncmove is one instruction and quite fast.

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

/*

//JSon string version of serial packet parser
void readSerialInput() { 
  static String inputLine = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      Serial.println("Attempting to parse Serial packet.");
      if (inputLine.startsWith("{")) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, inputLine);
        if (!error) {
          parseLegPath(doc);
          Serial.println("Finished Parsing.");
        } else {
          Serial.print("deserializeJson() failed: ");
          Serial.println(error.c_str());
        }
      }
      inputLine = "";
    } else {
      inputLine += c;
    }
  }
}

//JSon string version
void parseLegPath(JsonDocument& doc) {
  JsonObject msg = doc["msg"];

  for (int leg = 0; leg < 6; leg++) {
    char keyX[16];
    char keyY[16];
    char keyZ[16];
    sprintf(keyX, "path_l%d_x", leg);
    sprintf(keyY, "path_l%d_y", leg);
    sprintf(keyZ, "path_l%d_z", leg);

    JsonArray xArr = msg[keyX];
    JsonArray yArr = msg[keyY];
    JsonArray zArr = msg[keyZ];

    for (int step = 0; step < Steps; step++) {
      XPathBuf[leg][step] = xArr[step].as<float>();
      YPathBuf[leg][step] = yArr[step].as<float>();
      ZPathBuf[leg][step] = zArr[step].as<float>();
    }
  }

  JsonArray angArr = msg["path_ang"];
  for (int step = 0; step < Steps; step++) {
    TurnPathBuf[step] = angArr[step].as<float>();
  }

  dt = msg["dt"].as<float>();
  startSetPath = 1;
  standBegin = 1;
  newPathPending = true;      // start wait until current step ends
  Serial.println("Decoded a packet.");
}


void setup(){
  Serial.begin(9600);
  Serial.println("Starting up Sheldon's teensy ported to ROS2...");
  delay(1000);
}

double Angle[18];
double Spd[18];
uint8_t Id[18];
char dataStr[100] = "";
char buff[7];
long currentmillis = 0;
long prevmillis = millis();
float x=0.0;

//global variables for setnextpathpoint
long curtime = 0;
long prevtime = 0;

int stepStartFlag = 0;
int kinematicModeStartFlag = 0;

//Function prototype
void SetAngles(float* th1,float* th2,float* th3 ,float spd1=-1,float spd2=-1, float spd3=-1);

void loop(){
  readSerialInput(); 
  currentmillis = millis();
  
  if(currentmillis - prevmillis >= 10){
    prevmillis = currentmillis;
    // Serial.println("Loop running");
    // Serial.println(startUp);

    //tz = signal_generator(millis()/1000.0);
    //Serial.println("Signal Generator output:");
    //Serial.println(tz, 10);
    // Serial.println(micros());

    //On Startup
    if(startUp == 0)
    {
      //Serial.println("Running startup.");
      static long startUp_startTime = millis();
      InKin.IK(&theta1[0],&theta2[0],&theta3[0],283.71,   0.0,    -140,0,0,0,0,0,0);   //xyz are in world coords with leg 0 at 0 angle.
      InKin.IK(&theta1[1],&theta2[1],&theta3[1],141.855,  -245.7, -140,1,0,0,0,0,0);
      InKin.IK(&theta1[2],&theta2[2],&theta3[2],-141.855, -245.7, -140,2,0,0,0,0,0);
      InKin.IK(&theta1[3],&theta2[3],&theta3[3],-283.71,  0.0,    -140,3,0,0,0,0,0);
      InKin.IK(&theta1[4],&theta2[4],&theta3[4],-141.855, 245.7,  -140,4,0,0,0,0,0);
      InKin.IK(&theta1[5],&theta2[5],&theta3[5],141.855,  245.7,  -140,5,0,0,0,0,0);

      // Serial.println("Startup sequence joint positions in rad:");
      // for(int leg = 0; leg <6; leg++ ){
      //   Serial.print(theta1[leg]);
      //   Serial.print("  ");
      //   Serial.print(theta2[leg]);
      //   Serial.print("  ");
      //   Serial.print(theta3[leg]);
      //   Serial.println("  ");
      // }

      SetAngles(theta1,theta2,theta3,10,10,10);
      

      // Just copy the angles into an array and do a traditional syncmove for startup.
      
      // for(int i = 0; i < 6; i++){
      //   Angle[3*i  ] = theta1[i];
      //   Angle[3*i+1] = -theta2[i];
      //   Angle[3*i+2] = -theta3[i];
      // }
      // for(uint8_t i = 0; i < 18; i++){
      //   Id[i] = i;
      //   Spd[i] = 10.0;
      // }
      //dxl.SyncMove(Id,Angle,Spd,18);
      if(currentmillis - startUp_startTime >= 5000) 
      {
        startUp = 1;
        standBegin = 0;
        Serial.println("Finished Startup");
      }
    }

    //Teleop demo Mode
    if(mode == 0 && startUp == 1)
    {
      //Serial.println("Running teleop demo.");
      stepStartFlag = 0;
      static elapsedMillis kinematicModeStartTimer;

      if(kinematicModeStartFlag == 0)
      {
        kinematicModeStartFlag = 1;
        kinematicModeStartTimer = 0;
      }

      if(kinematicModeStartTimer <= 1000)
      {
        InKin.IK(&T_theta1[0],&T_theta2[0],&T_theta3[0],283.71+tx,   0.0+ty,    -140-tz,0,1000,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[1],&T_theta2[1],&T_theta3[1],141.855+tx,  -245.7+ty, -140-tz,1,1000,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[2],&T_theta2[2],&T_theta3[2],-141.855+tx, -245.7+ty, -140-tz,2,1000,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[3],&T_theta2[3],&T_theta3[3],-283.71+tx,  0.0+ty,    -140-tz,3,1000,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[4],&T_theta2[4],&T_theta3[4],-141.855+tx, 245.7+ty,  -140-tz,4,1000,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[5],&T_theta2[5],&T_theta3[5],141.855+tx,  245.7+ty,  -140-tz,5,1000,troll,tpitch,-tyaw,0);
        if(ConstrainCheck01(T_theta1,T_theta2,T_theta3))
        {
          SetAngles(T_theta1,T_theta2,T_theta3,10,10,10);
        }
      }
      else
      {
        InKin.IK(&T_theta1[0],&T_theta2[0],&T_theta3[0],283.71+tx,   0.0+ty,    -140-tz,0,0,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[1],&T_theta2[1],&T_theta3[1],141.855+tx,  -245.7+ty, -140-tz,1,0,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[2],&T_theta2[2],&T_theta3[2],-141.855+tx, -245.7+ty, -140-tz,2,0,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[3],&T_theta2[3],&T_theta3[3],-283.71+tx,  0.0+ty,    -140-tz,3,0,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[4],&T_theta2[4],&T_theta3[4],-141.855+tx, 245.7+ty,  -140-tz,4,0,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[5],&T_theta2[5],&T_theta3[5],141.855+tx,  245.7+ty,  -140-tz,5,0,troll,tpitch,-tyaw,0);
        if(ConstrainCheck01(T_theta1,T_theta2,T_theta3))
        {
          SetAngles(T_theta1,T_theta2,T_theta3,0,0,0);
          // Serial.println("Servo angle step check (cast to int):");
          //  Serial.println(T_theta3[0]/(0.29296875 * M_PI/180.0));
        }
      }
    }

    //SetAngle Mode
    else if(mode == 1 && startUp == 1)
    {
      Serial.println("Running SetAngle Mode.");
      stepStartFlag = 0;
      kinematicModeStartFlag = 0;
      
      A_theta1[0] = 0;
      A_theta2[0] = 0;
      A_theta3[0] = -0;

      float px = 0, py = 0, pz = 0;
      
      FK03_inbody(px,py,pz, A_theta1[0],A_theta2[0],A_theta3[0],0);
      InKin.IK(&A_theta1[0],&A_theta2[0],&A_theta3[0],px,   py,    pz,0,0,0,0,0,0);

      FK03_inbody(px,py,pz, A_theta1[1],A_theta2[1],A_theta3[1],1);
      InKin.IK(&A_theta1[1],&A_theta2[1],&A_theta3[1],px,   py,    pz,1,0,0,0,0,0);

      FK03_inbody(px,py,pz, A_theta1[2],A_theta2[2],A_theta3[2],2);
      InKin.IK(&A_theta1[2],&A_theta2[2],&A_theta3[2],px,   py,    pz,2,0,0,0,0,0);

      FK03_inbody(px,py,pz, A_theta1[3],A_theta2[3],A_theta3[3],3);
      InKin.IK(&A_theta1[3],&A_theta2[3],&A_theta3[3],px,   py,    pz,3,0,0,0,0,0);

      FK03_inbody(px,py,pz, A_theta1[4],A_theta2[4],A_theta3[4],4);
      InKin.IK(&A_theta1[4],&A_theta2[4],&A_theta3[4],px,   py,    pz,4,0,0,0,0,0);

      FK03_inbody(px,py,pz, A_theta1[5],A_theta2[5],A_theta3[5],5);
      InKin.IK(&A_theta1[5],&A_theta2[5],&A_theta3[5],px,   py,    pz,5,0,0,0,0,0);

      if(ConstrainCheck01(A_theta1,A_theta2,A_theta3))
      {
        SetAngles(A_theta1,A_theta2,A_theta3,20,20,20);
      }
      //mode = 2;
    }
    
    //SetNextpathPoint Mode
    else if(mode == 2 && startSetPath == 1 && startUp == 1)
    {
      
      kinematicModeStartFlag = 0;
      static elapsedMillis stepStartTimer = 1001;            //Note that this class auto increments 1000 times per second.
	    if (stepStartFlag == 0 && standBegin == 1)
	    {
        Serial.println("Starting SetNextpathPoint Mode");
	      stepStartTimer = 0;   // What do these two do? Something with the gait
	      stepStartFlag = 1;   
        standBegin = 0;      // We confirm that a new gait packet has arrived.
      }
      if(stepStartTimer <= 1000)
      {
        SetNextPathPoint(XPath,YPath,ZPath,TurnPath,1000);
        ConstrainCheck2(theta1,theta2,theta3);
        SetAngles(theta1,theta2,theta3,0,0,0);    //Spams position points at 100Hz. When spamming points. Speed is practically irrelevant. Just as fast as possible.
        prevtime = millis();
      }
      else
      {
        curtime = millis();
        SetNextPathPoint(XPath,YPath,ZPath,TurnPath,dt);
        ConstrainCheck2(theta1,theta2,theta3);
        SetAngles(theta1,theta2,theta3,0,0,0);
      }
    }
  }


}



// ** Functions **
// Not sure how this works
void SetNextPathPoint(float XP[][Pathsize*2-2],float YP[][Pathsize*2-2],float ZP[][Pathsize*2-2],float* TurnP,float d_t)
{
  static int currentPathPoint[6] = {3,9,3,9,3,9};
  static int currentPathPoint_tw[2] = {3,3};
  float x,y,z,yaww;

  if(d_t != -1000)
  {
    for(int i = 0;i<6;i++)
    {
      x = XP[i][currentPathPoint[i]];
      y = YP[i][currentPathPoint[i]];
      z = ZP[i][currentPathPoint[i]];
      yaww = TurnP[currentPathPoint_tw[i%2]];
  
      InKin.IK(&theta1[i],&theta2[i],&theta3[i],x,y,z,i,d_t,rollInput,pitchInput,yaww);
  
    }
  
    if(curtime - prevtime >= d_t)
    {
      prevtime = curtime;
  
      for(int i=0;i<2;i++)
      {
        if(currentPathPoint_tw[i] >= 11)
        {
          currentPathPoint_tw[i] = 0;
        }
        else
        {
          currentPathPoint_tw[i] = currentPathPoint_tw[i] + 1;
        }
      }
      
      for(int i=0;i<6;i++)
      {
        if(currentPathPoint[i] >= 11)
        {
          currentPathPoint[i] = 0;
        }
        else
        {
          currentPathPoint[i] = currentPathPoint[i] + 1;
        }
      }

      if(currentPathPoint[0] == 7 || currentPathPoint[1] == 7)
      {
        //FeetOnFloor.data = 1;
        FeetOnFloor = 1;
        //pub_FeetOnFloorFlag.publish(&FeetOnFloor); 
      }

      if (currentPathPoint[0] == 0 && newPathPending) {                 //Switch to the new gait here if it is availble
        memcpy(XPath, XPathBuf, sizeof(XPath));
        memcpy(YPath, YPathBuf, sizeof(YPath));
        memcpy(ZPath, ZPathBuf, sizeof(ZPath));
        memcpy(TurnPath, TurnPathBuf, sizeof(TurnPath));
        newPathPending = false;

        Serial.println("Switched to new gait at end of cycle.");
      }
    }
  }
  else
  {
    for(int i = 0;i<6;i++)
    {
      currentPathPoint_tw[i%2] = 3;
      currentPathPoint[i] = (i%2 == 0) ? 3 : 9;
    }
    prevtime = curtime;
    stepStartFlag = 0;

    if(standBegin == 0)
    {
      for(int i = 0;i<6;i++)
      {
        x = XP[i][currentPathPoint[i]];
        y = YP[i][currentPathPoint[i]];
        z = -140.0;
        yaww = TurnP[currentPathPoint_tw[i%2]];
    
        InKin.IK(&theta1[i],&theta2[i],&theta3[i],x,y,z,i,1000,rollInput,pitchInput,yaww);
      }
    }
  }
}

float AngleSmoothPrev = 150;
float AngleSmooth = 0;

void SetAngles(float* th1,float* th2,float* th3 ,float spd1,float spd2, float spd3)
{

  //static double prevAngle[18];
  //long milli = millis();
  String mfb0,mfb1,mfb2,mfb3,mfb4,mfb5,mfb6,mfb7,mfb8,mfb9,mfb10,mfb11,mfb12,mfb13,mfb14,mfb15,mfb16,mfb17;
  
  //void SetAngles(float* th1,float* th2,float* th3 ,float spd1,float spd2, float spd3)
  for(int i = 0; i < 6; i++){
        Angle[3*i  ] =  th1[i];
        Angle[3*i+1] = -th2[i];
        Angle[3*i+2] = -th3[i];

        Spd[3*i  ] = spd1;
        Spd[3*i+1] = spd2;
        Spd[3*i+2] = spd3;

         // Print angles
    Serial.print("i = ");
    Serial.print(i);
    Serial.print(" | th1: ");
    Serial.print(th1[i]);
    Serial.print(" | th2: ");
    Serial.print(th2[i]);
    Serial.print(" | th3: ");
    Serial.print(th3[i]);

    // Print speeds
    Serial.print(" | spd1: ");
    Serial.print(spd1);
    Serial.print(" | spd2: ");
    Serial.print(spd2);
    Serial.print(" | spd3: ");
    Serial.println(spd3);
      }
      for(uint8_t i = 0; i < 18; i++){
        Id[i] = i;
        //Spd[i] = 10.0;
      }


  dxl.SyncMove(Id,Angle,Spd,18);
  
  //pubAngleFB(mfb0,mfb1,mfb2,mfb3,mfb4,mfb5,mfb6,mfb7,mfb8,mfb9,mfb10,mfb11,mfb12,mfb13,mfb14,mfb15,mfb16,mfb17);
}

int ConstrainCheck01(float* th1,float* th2,float* th3)
{
  int LegitMove = 1;

  for(int i = 0;i<6;i++)
  {
    if((th3[i] > 0.0) || (th3[i] < -150.0/180.0*M_PI))
    {
      LegitMove = 0;
    }

    if((th2[i] > 60.0/180.0*M_PI) || (th2[i] < -90.0/180.0*M_PI))
    {
      LegitMove = 0;
    }

    if((th1[i] > 50.0/180.0*M_PI) || (th1[i] < -50.0/180.0*M_PI))
    {
      LegitMove = 0;
    }
  }
  return LegitMove;
}

void ConstrainCheck2(float* th1,float* th2,float* th3)
{
  static float prevTh1[6] = {0,0,0,0,0,0};
  static float prevTh2[6] = {0,0,0,0,0,0};
  static float prevTh3[6] = {0,0,0,0,0,0};
  float px = 0, py = 0, pz = 0;

  int LegitMove = 1;

  for(int i = 0;i<6;i++)
  {
    if((th3[i] > 0.0) || (th3[i] < -150.0/180.0*M_PI))
    {
      LegitMove = 0;
    }

    if((th2[i] > 60.0/180.0*M_PI) || (th2[i] < -90.0/180.0*M_PI))
    {
      LegitMove = 0;
    }

    if((th1[i] > 50.0/180.0*M_PI) || (th1[i] < -50.0/180.0*M_PI))
    {
      LegitMove = 0;
    }
  }

  if(LegitMove == 1)
  {
    for(int i = 0;i<6;i++)
    {
      prevTh1[i] = th1[i];
      prevTh2[i] = th2[i];
      prevTh3[i] = th3[i];
    }
  }
  else if(LegitMove == 0)
  {
    for(int i = 0;i<6;i++)
    {
      th1[i] = prevTh1[i];
      th2[i] = prevTh2[i];
      th3[i] = prevTh3[i];

      FK03_inbody(px,py,pz, th1[i],th2[i],th3[i],i);
      InKin.IK(&th1[i],&th2[i],&th3[i],px,   py,    pz,i,0,0,0,0,0);
    }
  }
}

void pubAngleFB(String &mfb0,String &mfb1,String &mfb2,String &mfb3,String &mfb4,String &mfb5,String &mfb6,String &mfb7,String &mfb8,String &mfb9,String &mfb10,String &mfb11,String &mfb12,String &mfb13,String &mfb14,String &mfb15,String &mfb16,String &mfb17)
{
  String feed = mfb0+ ","+mfb1+ ","+mfb2+ ","+mfb6+ ","+mfb7+ ","+mfb8+ ","+mfb12+ ","+mfb13+ ","+mfb14;
  Serial.println(feed);  
}

double signal_generator(double time) {
    //for triangular signals to test dynamics
    const float freq = 1;          // Frequency in Hz
    const float amplitude = 20.0;      // =1, Peak amplitude (from -1 to 1)
    const float offset = 0.0;         // DC offset

    float period = 1.0 / freq;
    float t = fmod(time, period);     // Keep time in current period
    float slope = (4.0 * amplitude) / period;

    if (t < period / 2.0) {
        return offset + (slope * t - amplitude);
    } else {
        return offset + (-slope * t + 3 * amplitude);
    }
}

*/

