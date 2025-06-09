#ifndef MUJOCO_HIL_SIM_H
#define MUJOCO_HIL_SIM_H

// Handle the interface between the Teensy logic and the Mujoco real-time hardware-in-the-loop simulation over the serial port.
// Read servo positions for feedback control and force detection and write to servos to command positions.
// Place next similar functions which interface with the real hardware servos.
// Note that the python script driving the mujoco simulation should match this interface. Number of bytes sent, sync bytes, etc.

#include <Arduino.h>

class MujocoClass {
public:
  void writeServoSim(double angles[18], double spd);
  bool readServoSim(double angles[18]);
};

extern MujocoClass Mujoco;

#endif