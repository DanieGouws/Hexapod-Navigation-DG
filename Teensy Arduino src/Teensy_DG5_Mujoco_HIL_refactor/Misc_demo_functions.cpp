#include "Misc_demo_functions.h"

// Only for creating a square walking demo.
// Returns the (x, y) coordinate on the square for step_index
Vector2f DemoClass::getSquareStepPoint(int index) {
  // index = index % NUM_STEPS;  // Wrap around
  double total_len = STEP_SIZE * index;

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

DemoClass Demo = DemoClass();