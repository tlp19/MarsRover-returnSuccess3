#ifndef coordinate_instructions_h
#define coordinate_instructions_h

#include <math.h>

//Coordinates and orientation of the rover
long theta = 0;
float last_x = 0;
float last_y = 0;

//Common variables for functions
long L = 0;
long angle = 0;


//Translate coordinate instruction into distance and angle parameters
void translateCoordinates(long x, long y) {
  Serial.println("x = " + String(last_x));
  Serial.println("y = " + String(last_y));
  Serial.println("theta = " + String(theta));

  //compute the distance that we need to travel
  L = sqrt((((x) - (last_x)) * ((x) - (last_x))) + (((y) - (last_y)) * ((y) - (last_y))));
  Serial.println("L = " + String(L));

  //compute the angle that we need to turn by
  if ((x == last_x) && (y == last_y)) {
    angle = 0;
    Serial.println(0);
  }
  else if (((x >= last_x) && (y >= last_y)) || ((x <= last_x) && (y >= last_y))) {
    float arg = ((x - last_x) / L);
    if (arg > 1) {
      arg = arg - 1;
      angle = (asin(arg) + PI / 2 - theta * PI / 180L) * 180L / PI;
    } else if (arg < -1) {
      arg = arg + 1;
      angle = (asin(arg) - PI / 2 - theta * PI / 180L) * 180L / PI;
    }
    else {
      angle = (asin(arg) - theta * PI / 180L) * 180L / PI;
    }
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
    Serial.println(1);
  }
  else if ((x >= last_x) && (y <= last_y)) {
    float arg = ((x - last_x) / L);
    if (arg > 1) {
      arg = arg - 1;
      angle = (PI - asin(arg) + PI / 2 - theta * PI / 180L) * 180L / PI;
    } else if (arg < -1) {
      arg = arg + 1;
      angle = (PI - asin(arg) - PI / 2 - theta * PI / 180L) * 180L / PI;
    }
    else {
      angle = (PI - asin(arg) - theta * PI / 180L) * 180L / PI;
    }
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
    Serial.println(2);
  }
  else if ((x <= last_x) && (y <= last_y)) {
    float arg = ((x - last_x) / L);
    if (arg > 1) {
      arg = arg - 1;
      angle = (- PI - asin(arg) + PI / 2 - theta * PI / 180L) * 180L / PI;
    } else if (arg < -1) {
      arg = arg + 1;
      angle = (- PI - asin(arg) - PI / 2 - theta * PI / 180L) * 180L / PI;
    }
    else {
      angle = (-PI - asin(arg) - theta * PI / 180L) * 180L / PI;
    }
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
    Serial.println(3);
  }
  Serial.println("angle = " + String(angle));
}

#endif
