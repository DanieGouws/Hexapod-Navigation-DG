#ifndef MISC_DEMO_FUNCTIONS_H
#define MISC_DEMO_FUNCTIONS_H

// Functions useful to demo the hexapod by replacing components which should be handled by ros2. 
// Mostly High-level path generation and resulting foot target coordinate generation.

#include <ArduinoEigen.h>
using namespace Eigen;

//walking demo
const double STEP_SIZE = 50.0f;     // 5 cm per step
const double SQUARE_SIZE = 500.0f;   // 0.5 meter square
const int NUM_STEPS = 4 * (int)(SQUARE_SIZE / STEP_SIZE);  // 40 steps

class DemoClass{
  public:
    Vector2f getSquareStepPoint(int index);
};

extern DemoClass Demo;

#endif