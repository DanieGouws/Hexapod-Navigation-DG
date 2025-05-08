#ifndef MyDynamixel_h
#define MyDynamixel_h

#include "Arduino.h"

// In radians
const float SERVO_STEP = 0.29296875 * M_PI/180;

//Possible instructions for a motor
enum INSTRUCTION 
{
  PING = 0x01, READ_DATA = 0x02, WRITE_DATA = 0x03, REG_WRITE = 0x04, ACTION = 0x05, SYNC_WRITE = 0x83
};

//Control table for motors
enum CONTROL_TABLE
{
  MODEL_NUMBER_L, // 0x00
  MODEL_NUMBER_H, // 0x01
  VERSION, // 0x02
  ID, // 0x03
  BAUD_RATE, // 0x04
  RETURN_DELAY_TIME, // 0x05
  CW_ANGLE_LIMIT_L, // 0x06
  CW_ANGLE_LIMIT_H, // 0x07
  CCW_ANGLE_LIMIT_L, // 0x08
  CCW_ANGLE_LIMIT_H, // 0x09
  RESERVED1, // 0x0A
  HIGHEST_LIMIT_TEMPERATURE, // 0x0B
  LOWEST_LIMIT_VOLTAGE, // 0x0C
  HIGHEST_LIMIT_VOLTAGE, // 0x0D
  MAX_TORQUE_L, // 0x0E
  MAX_TORQUE_H, // 0x0F
  STATUS_RETURN_LEVEL, // 0x10
  ALARM_LED, // 0x11
  ALARM_SHUTDOWN, // 0x12
  RESERVED2, // 0x13
  RESERVED3, // 0x14
  RESERVED4, // 0x15
  RESERVED5, // 0x16
  RESERVED6, // 0x17
  TORQUE_ENABLE, // 0x18
  LED, // 0x19
  CW_COMPLIANCE_MARGIN, // 0x1A
  CCW_COMPLIANCE_MARGIN, // 0x1B
  CW_COMPLIANCE_SLOPE, // 0x1C
  CCW_COMPLIANCE_SLOPE, // 0x1D
  GOAL_POSITION_L, // 0x1E
  GOAL_POSITION_H, // 0x1F
  MOVING_SPEED_L, // 0x20
  MOVING_SPEED_H, // 0x21
  TORQUE_LIMIT_L, // 0x22
  TORQUE_LIMIT_H, // 0x23
  PRESENT_POSITION_L, // 0x24
  PRESENT_POSITION_H, // 0x25
  PRESENT_SPEED_L, // 0x26
  PRESENT_SPEED_H, // 0x27
  PRESENT_LOAD_L, // 0x28
  PRESENT_LOAD_H, // 0x29
  PRESENT_VOLTAGE, // 0x2A
  PRESENT_TEMPERATURE, // 0x2B
  REGISTERED_INSTRUCTION, // 0x2C
  RESERVE3, // 0x2D
  MOVING, // 0x2E
  LOCK, // 0x2F
  PUNCH_L, // 0x30
  PUNCH_H // 0x31
};

class MyDynamixel
{
  public:
    MyDynamixel(HardwareSerial& port, int rate, int Dir_pin);
    void WriteServos (uint8_t id,enum CONTROL_TABLE reg, uint8_t * data, uint8_t dataLength);
    void ReadServos (uint8_t id,enum CONTROL_TABLE reg, uint8_t dataLength, uint8_t* returnPack);
	  void MoveServos (uint8_t id, double position, double speed = -10);
	  double PresentPos (uint8_t id);
    void SyncDisableTorque(uint8_t* id, uint8_t numMotors);
	  void SyncWrite (uint8_t* id, uint8_t numMotors, enum CONTROL_TABLE reg, uint8_t * data, uint8_t dataLength);
	  void SyncMove (uint8_t* id, double* position, double* speed, uint8_t numMotors);
  
  private:
	  uint8_t goalMovementBits[4] = {0,0,0,0};
	  HardwareSerial& _port;
	  int dir_pin_;
};

#endif
