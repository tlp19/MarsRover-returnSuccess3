#ifndef coordinate_instructions_h
#define coordinate_instructions_h

#include "instructionQueue.h"
#include <math.h>

const float pi = 3.14159265359;

//Common variables for both functions
long L;
long angle;

//Internal state variables for each interation of function
long theta = 0;
long last_x = 0;
long last_y = 0;
float b_x = 0;
float b_y = 0;
float sum_b_x = 0;
float sum_b_y = 0;

//Translate coordinate instruction into distance and angle parameters
void translateCoordinates(long x, long y) {

  //find the distance to travel by
  L = sqrt((((x) - (last_x)) * ((x) - (last_x))) + (((y) - (last_y)) * ((y) - (last_y))));

  //find the angle to turn by
  if ((x == last_x) && (y == last_y)) {
    angle = 0;
  } else if ((x >= last_x) && (y > last_y)) {
    angle = (asin(((x - sum_b_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
  }
  else if ((x >= last_x) && (y < last_y)) {
    angle = (pi - asin(((x - sum_b_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = abs(angle) - 360;
    }
  }
  else if ((x >= last_x) && (y == last_y)) {
    angle = (acos(((y - sum_b_y) / L)) - theta * pi / 180L) * 180L / pi;
  }
  else if ((x <= last_x) && (y > last_y)) {
    angle = (asin(((x - sum_b_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
  }
  else if ((x <= last_x) && (y < last_y)) {
    angle = (-pi - asin(((x - sum_b_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
  }
  else if ((x <= last_x) && (y == last_y)) {
    angle = ((acos(((y - sum_b_y) / L)) - theta * pi / 180L) - pi) * 180L / pi;
  }

  //update stored last coordinates
  last_x = x;
  last_y = y;
  //update internal parameters
  theta += angle;
  b_x = L * sin(theta * pi / 180L);
  b_y = L * cos(theta * pi / 180L);
  sum_b_x += b_x;
  sum_b_y += b_y;


}


//Constructs two instructions based on given coordinates
void buildQueueInstructions(long x, long y) {
  //first find equivalent distance and angle
  translateCoordinates(x, y);

  //build turn instruction
  Instruction turnInstruction;
  if (angle != 0) {
    if (angle >= 0) {
      turnInstruction = {"CL", String(angle)};
      instructionQueue.push(turnInstruction);
    } else {
      turnInstruction = {"CC", String(-angle)};
      instructionQueue.push(turnInstruction);
    }
  }
  //build forward instruction
  if (L != 0) {
    Instruction forwardInstruction = {"FW", String(L)};
    instructionQueue.push(forwardInstruction);
  }
}

#endif
