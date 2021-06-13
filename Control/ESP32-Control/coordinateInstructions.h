#ifndef coordinate_instructions_h
#define coordinate_instructions_h

#include <math.h>

const float pi = 3.14159265359;

//Common variables for both functions
long L = 0;
long angle = 0;
long theta = 0;

//Internal state variables for each interation of function
long last_x = 0;
long last_y = 0;
float b_x = 0;
float b_y = 0;
float sum_b_x = 0;
float sum_b_y = 0;

//Translate coordinate instruction into distance and angle parameters
void translateCoordinates(long x, long y) {

  b_x = L * sin((theta) * pi / 180L);
  b_y = L * cos((theta) * pi / 180L);
  sum_b_x += b_x;
  sum_b_y += b_y;

  //find the distance to travel by
  L = sqrt((((x) - (last_x)) * ((x) - (last_x))) + (((y) - (last_y)) * ((y) - (last_y))));

  //find the angle to turn by
  if ((x == last_x) && (y == last_y)) {
    angle = 0;
    Serial.println(0);
  }
  else if ((x >= last_x) && (y > last_y)) {
    angle = (asin(((x - sum_b_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
    Serial.println(1);
  }
  else if ((x >= last_x) && (y < last_y)) {
    angle = (pi - asin(((x - sum_b_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = abs(angle) - 360;
    }
    Serial.println(2);
  }
  else if ((x >= last_x) && (y == last_y)) {
    angle = (acos(((y - sum_b_y) / L)) - theta * pi / 180L) * 180L / pi;
    Serial.println(3);
  }
  else if ((x <= last_x) && (y > last_y)) {
    angle = (asin(((x - sum_b_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
    Serial.println(4);
  }
  else if ((x <= last_x) && (y < last_y)) {
    angle = (-pi - asin(((x - sum_b_x) / L)) - theta * pi / 180L) * 180L / pi;
    if (abs(angle) > 180) {
      angle = 360 - abs(angle);
    }
    Serial.println(5);
  }
  else if ((x <= last_x) && (y == last_y)) {
    angle = ((acos(((y - sum_b_y) / L)) - theta * pi / 180L) - pi) * 180L / pi;
    Serial.println(6);
  }

}

#endif
