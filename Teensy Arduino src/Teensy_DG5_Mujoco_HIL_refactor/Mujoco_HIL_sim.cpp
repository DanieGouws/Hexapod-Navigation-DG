#include "Mujoco_HIL_sim.h"

//Write commanded servo positions to the Mujoco simulation via Serial port
void MujocoClass::writeServoSim(double angles[18], double spd){
   const int total_bytes = 18 * 8;  //sizeof(angles)

  // Define a sync byte to mark start of message
  const uint8_t sync = 0xAB;
  Serial.write(sync);

  // Send angles first
  Serial.write((uint8_t*)angles, total_bytes);

  // Optionally send speeds
  double speeds[18] = {spd};
  Serial.write((uint8_t*)speeds, total_bytes);
}

bool MujocoClass::readServoSim(double angles[18]) {
  // Flush buffer
  while (Serial.available()) Serial.read();

  // Send request
  Serial.write(0xCE);

  const int total_bytes = 18 * 8;  //sizeof(angles)
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

  // Convert byte buffer into doubles
  memcpy(angles, buf, total_bytes);  // byte-safe bulk copy
  return true;
}

MujocoClass Mujoco;