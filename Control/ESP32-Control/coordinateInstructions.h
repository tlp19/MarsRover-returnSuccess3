#ifndef coordinate_instructions_h
#define coordinate_instructions_h

//#include "instructionQueue.h"
#include <math.h>

const float pi = 3.14159265359;

//Common variables for both functions
long L = 0;
long angle = 0;
long theta = 0;

//Current coordinates of rover
long last_x = 0;
long last_y = 0;

//Translate coordinate instruction into distance and angle parameters
void translateCoordinates(long x, long y) {

  //find the distance to travel by
  L = sqrt((((x) - (last_x)) * ((x) - (last_x))) + (((y) - (last_y)) * ((y) - (last_y))));

  //find the angle to turn by
  if ((x == last_x) && (y == last_y)) {
    angle = 0;
  }
  else if ((x >= last_x) && (y > last_y)) {
    angle = (asin(((x - last_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
  }
  else if ((x >= last_x) && (y < last_y)) {
    angle = (pi - asin(((x - last_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = abs(angle) - 360;
    }
  }
  else if ((x >= last_x) && (y == last_y)) {
    angle = (acos(((y - last_y) / L)) - theta * pi / 180L) * 180L / pi;
  }
  else if ((x <= last_x) && (y > last_y)) {
    angle = (asin(((x - last_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
  }
  else if ((x <= last_x) && (y < last_y)) {
    angle = (-pi - asin(((x - last_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
  }
  else if ((x <= last_x) && (y == last_y)) {
    angle = ((acos(((y - last_y) / L)) - theta * pi / 180L) - pi) * 180L / pi;
  }

}

#endif
