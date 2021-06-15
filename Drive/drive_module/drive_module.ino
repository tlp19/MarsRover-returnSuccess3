
#include <Wire.h>
#include <stdint.h>
#include <INA219_WE.h>
#include "SPI.h"

INA219_WE ina219; // this is the instantiation of the library for the current sensor


#define RX0 2
#define TX0 1

#define PIN_SS        10
#define PIN_MISO      12
#define PIN_MOSI      11
#define PIN_SCK       13

#define PIN_MOUSECAM_RESET     8
#define PIN_MOUSECAM_CS        7

#define ADNS3080_PIXELS_X                30
#define ADNS3080_PIXELS_Y                30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

#define ADNS3080_PRODUCT_ID_VAL        0x17



float open_loop, closed_loop; // Duty Cycles
float vpd, vb, vref, iL, dutyref, current_mA; // Measurement Variables
unsigned int sensorValue0, sensorValue1, sensorValue3; // ADC sample values declaration

unsigned int sensorValue2 = 800; // velocity control initialization

float ev = 0, cv = 0, ei = 0, oc = 0; //internal signals
float Ts = 0.0008; //1.25 kHz control frequency. It's better to design the control period as integral multiple of switching period.
float kpv = 0.05024, kiv = 15.78, kdv = 0; // voltage pid.
float u0v, u1v, delta_uv, e0v, e1v, e2v; // u->output; e->error; 0->this time; 1->last time; 2->last last time
float kpi = 0.02512, kii = 39.4, kdi = 0; // current pid.
float u0i, u1i, delta_ui, e0i, e1i, e2i; // Internal values for the current controller
float uv_max = 4, uv_min = 0; //anti-windup limitation
float ui_max = 1, ui_min = 0; //anti-windup limitation
float current_limit = 1.0;
boolean Boost_mode = 0;
boolean CL_mode = 0;


unsigned int loopTrigger;
unsigned int com_count = 0; // a variables to count the interrupts. Used for program debugging.


//************************** Motor Constants **************************//
unsigned long previousMillis = 0; //initializing time counter

int DIRRstate = LOW;              //initializing direction states
int DIRLstate = LOW;

int DIRL = 20;                    //defining left direction pin
int DIRR = 21;                    //defining right direction pin

int pwmr = 5;                     //pin to control right wheel speed using pwm
int pwml = 9;                     //pin to control left wheel speed using pwm


//***************** MOTOR Commands for communication **************************//
char command;
long d;

bool UARTdataSent = true;

// *************** Initializing variables foordinates *********************//
long L;
long angle;
long theta = 0;

long last_x = 0;     // initializing at x = 0
long last_y = 0;     // initializing at y = 0


//***************************** OPTICAL SENSOR **************************************//

long total_x = 0;
long total_y = 0;

long total_x1 = 0;
long total_y1 = 0;

long x = 0;
long y = 0;

long a = 0;
long b = 0;

long distance_x = 0;
long distance_y = 0;

volatile byte movementflag = 0;
volatile int xydat[2];

int convTwosComp(int b) {
  //Convert from 2's complement
  if (b & 0x80) {
    b = -1 * ((b ^ 0xff) + 1);
  }
  return b;
}


int tdistance = 0;


void mousecam_reset()
{
  digitalWrite(PIN_MOUSECAM_RESET, HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET, LOW);
  delay(35); // 35ms from reset to functional
}


int mousecam_init()
{
  pinMode(PIN_MOUSECAM_RESET, OUTPUT);
  pinMode(PIN_MOUSECAM_CS, OUTPUT);

  digitalWrite(PIN_MOUSECAM_CS, HIGH);

  mousecam_reset();

  int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
  if (pid != ADNS3080_PRODUCT_ID_VAL)
    return -1;

  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);

  return 0;
}

void mousecam_write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(50);
}

int mousecam_read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(1);
  return ret;
}

struct MD {
  byte motion;
  char dx, dy;
  byte squal;
  word shutter;
  byte max_pix;
};


void mousecam_read_motion(struct MD *p)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion =  SPI.transfer(0xff);
  p->dx =  SPI.transfer(0xff);
  p->dy =  SPI.transfer(0xff);
  p->squal =  SPI.transfer(0xff);
  p->shutter =  SPI.transfer(0xff) << 8;
  p->shutter |=  SPI.transfer(0xff);
  p->max_pix =  SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(5);
}

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(byte *pdata)
{
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE, 0x83);

  digitalWrite(PIN_MOUSECAM_CS, LOW);

  SPI.transfer(ADNS3080_PIXEL_BURST);
  delayMicroseconds(50);

  int pix;
  byte started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for (count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y; )
  {
    pix = SPI.transfer(0xff);
    delayMicroseconds(10);
    if (started == 0)
    {
      if (pix & 0x40)
        started = 1;
      else
      {
        timeout++;
        if (timeout == 100)
        {
          ret = -1;
          break;
        }
      }
    }
    if (started == 1)
    {
      pdata[count++] = (pix & 0x3f) << 2; // scale to normal grayscale byte range
    }
  }

  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(14);

  return ret;
}

void setup() {

  //************************** Motor Pins Defining **************************//
  pinMode(DIRR, OUTPUT);
  pinMode(DIRL, OUTPUT);
  pinMode(pwmr, OUTPUT);
  pinMode(pwml, OUTPUT);
  digitalWrite(pwmr, HIGH);       //setting right motor speed at maximum
  digitalWrite(pwml, HIGH);       //setting left motor speed at maximum


  //************************** Optical Sensor Pins Defining **************************//
  pinMode(PIN_SS, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  Serial.begin(115200);
  Serial1.begin(9600);

  if (mousecam_init() == -1)
  {
    Serial.println("Mouse cam failed to init");
    while (1);
  }
  //*******************************************************************//


  //Basic pin setups

  noInterrupts(); //disable all interrupts
  pinMode(13, OUTPUT);       // Pin13 is used to time the loops of the controller
  pinMode(3, INPUT_PULLUP);  // Pin3 is the input from the Buck/Boost switch
  pinMode(2, INPUT_PULLUP);  // Pin 2 is the input from the CL/OL switch
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  // TimerA0 initialization for control-loop interrupt.

  TCA0.SINGLE.PER = 999;  //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = (TCA_SINGLE_CLKSEL_DIV16_gc) | (TCA_SINGLE_ENABLE_bm); //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm;

  // TimerB0 initialization for PWM output

  pinMode(6, OUTPUT);
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz
  analogWrite(6, 120);

  interrupts();  //enable interrupts.
  Wire.begin();  // We need this for the i2c comms for the current sensor
  ina219.init(); // this initiates the current sensor
  Wire.setClock(700000); // set the comms speed for i2c

  Serial1.println('d');
}

char asciiart(int k) {
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k >> 4];
}
byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];


void loop() {
#if 0

  /*
    if(movementflag){

    tdistance = tdistance + convTwosComp(xydat[0]);
    Serial.println("Distance = " + String(tdistance));
    movementflag=0;
    delay(3);
    }
  */

  // if enabled this section grabs frames and outputs them as ascii art

  if (mousecam_frame_capture(frame) == 0)
  {
    long i, j, k;
    for (i = 0, k = 0; i < ADNS3080_PIXELS_Y; i++)
    {
      for (j = 0; j < ADNS3080_PIXELS_X; j++, k++)
      {
        Serial.print(asciiart(frame[k]));
        Serial.print(' ');
      }
      Serial.println();
    }
  }
  Serial.println();
  //delay(250);

#else



  // if enabled this section produces a bar graph of the surface quality that can be used to focus the camera
  // also drawn is the average pixel value 0-63 and the shutter speed and the motion dx,dy.

  long val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  MD md;
  mousecam_read_motion(&md);
  for (long i = 0; i < md.squal / 4; i++)
    Serial.print('*');
  Serial.print(' ');
  Serial.print((val * 100) / 351);
  Serial.print(' ');
  Serial.print(md.shutter); Serial.print(" (");
  Serial.print((long)md.dx); Serial.print(',');
  Serial.print((long)md.dy); Serial.println(')');

  //debug



  // Serial.println(md.max_pix);
  //delay(100);

  distance_x = md.dx; //convTwosComp(md.dx);
  distance_y = md.dy; //convTwosComp(md.dy);

  total_x1 = (total_x1 + distance_x);
  total_y1 = (total_y1 + distance_y);

  total_x = 10 * total_x1 / 157; //Conversion from counts per inch to mm (400 counts per inch)
  total_y = 10 * total_y1 / 157; //Conversion from counts per inch to mm (400 counts per inch)


  Serial.print('\n');
  Serial.println("Distance_x = " + String(total_x));
  Serial.println("Distance_y = " + String(total_y));
  Serial.print('\n');
  //delay(100);

#endif


  if (loopTrigger) { // This loop is triggered, it wont run unless there is an interrupt

    digitalWrite(13, HIGH);   // set pin 13. Pin13 shows the time consumed by each control cycle. It's used for debugging.

    // Sample all of the measurements and check which control mode we are in
    sampling();
    CL_mode = digitalRead(3);    // input from the OL_CL switch
    Boost_mode = digitalRead(2); // input from the Buck_Boost switch

    if (Boost_mode) {
      if (CL_mode) { //Closed Loop Boost
        pwm_modulate(1); // This disables the Boost as we are not using this mode
      } else { // Open Loop Boost
        pwm_modulate(1); // This disables the Boost as we are not using this mode
      }
    } else {
      if (CL_mode) { // Closed Loop Buck
        // The closed loop path has a voltage controller cascaded with a current controller. The voltage controller
        // creates a current demand based upon the voltage error. This demand is saturated to give current limiting.
        // The current loop then gives a duty cycle demand based upon the error between demanded current and measured
        // current
        current_limit = 3; // Buck has a higher current limit
        ev = vref - vb;  //voltage error at this time
        cv = pidv(ev); //voltage pid
        cv = saturation(cv, current_limit, 0); //current demand saturation
        ei = cv - iL; //current error
        closed_loop = pidi(ei); //current pid
        closed_loop = saturation(closed_loop, 0.99, 0.01); //duty_cycle saturation
        pwm_modulate(closed_loop); //pwm modulation
      } else { // Open Loop Buck
        current_limit = 3; // Buck has a higher current limit
        oc = iL - current_limit; // Calculate the difference between current measurement and current limit
        if ( oc > 0) {
          open_loop = open_loop - 0.001; // We are above the current limit so less duty cycle
        } else {
          open_loop = open_loop + 0.001; // We are below the current limit so more duty cycle
        }
        open_loop = saturation(open_loop, dutyref, 0.02); // saturate the duty cycle at the reference or a min of 0.01
        pwm_modulate(open_loop); // and send it out
      }
    }
    // closed loop control path

    digitalWrite(13, LOW);   // reset pin13.
    loopTrigger = 0;
  }

  //****** ACTIONS ******//

  receiveData();
  directionControl(command, d);
  stopRoverOnCommand();

  //Serial.println("command = " + String(command));
  //Serial.println("d = " + String(d));

  //Serial.println("x-coordinates: " + String(last_x));
  //Serial.println("y-coordinates: " + String(last_y));

}


// receiving data
void receiveData() {
  String receivedChars = "";
  bool recvInProgress = false;
  int index = 0;
  char endMarker = '\r';
  byte incoming_char;

  while (Serial1.available()) {
    recvInProgress = true;
    Serial.print("Received from Control: ");
    incoming_char = Serial1.read();
    Serial.println(incoming_char);

    if (incoming_char != endMarker) {
      if (incoming_char > 57) {
        command = incoming_char;
        Serial.print(" -> command    | ");
        Serial.println(index);
      } else {
        receivedChars += (char)incoming_char;
        Serial.print(" -> appended to d: ");
        Serial.println(receivedChars);
      }
    }
  }
  if (recvInProgress) {
    receivedChars.trim();
    d = receivedChars.toInt();
  }
}

//************************** Direction Control ***************************//

void directionControl(char command, long d_a) {

  // moving forwards
  if (command == 'f') {
    // initialize speed when it moves foward
    digitalWrite(pwmr, HIGH);   //setting right motor speed at maximum
    digitalWrite(pwml, HIGH);   //setting left motor speed at maximum
    int y = d_a;
    // compare sensor's y-coordinates with the desired distance
    if (y - abs(total_y) < 2) { //use absolute value to compute the difference between to positive values
      stopRover();
      L = abs(total_y);     // distance travelled 
      sendCoordinates();
      command = '\0';       // reseting the instruction 
      d = 0;
      total_x1 = 0;         // resetting the sensor's coordinates
      total_y1 = 0;
      distance_x = 0;
      distance_y = 0;
    } else {
      // if y-coordinates do not match with the desired distance, then move foward
      DIRRstate = LOW;
      DIRLstate = LOW;
      UARTdataSent = false; // boolean flag set to false as rover is still moving
    }
  }

  // rotating clockwise
  else if (command == 'r') {
    digitalWrite(pwmr, HIGH);   
    digitalWrite(pwml, HIGH);   
    // add offset to compensate for sensor innacuracy
    long a = d_a + ((d_a * (float)20) / (float)360);  
    // compare the computed angle from sensor's x-coordinates with the desired angle
    if (a - ((abs(total_x) * (float)180) / ((float)135 * PI)) < 1) {
      stopRover();
      if (!UARTdataSent) {
        // send the angle back to control: need to remove the offset from the measured angle 
        angle = (((abs(total_x) * (float)180) / ((float)135 * PI)) - ((d_a * (float)20) / (float)360) + (float)1);
        theta += angle; //the angle should be added to the previous angles in order to compute the current coordinates
        Serial1.print('a');
        Serial1.println(String(theta));
        Serial1.println('d');
        UARTdataSent = true;      // flags allows to only execute the loop once
      }
      command = '\0';
      d = 0;
      total_x1 = 0;
      total_y1 = 0;
      distance_x = 0;
      distance_y = 0;
    } else {
      // if the desired angle does not match the computed angle, then move turn clockwise
      DIRRstate = HIGH;
      DIRLstate = LOW;
      UARTdataSent = false;
    }
  }
  // rotating counterclockwise
  else if (command == 'l') {
    digitalWrite(pwmr, HIGH);   
    digitalWrite(pwml, HIGH);   
    long a = d_a + ((d_a * (float)20) / (float)360); // add offset to compensate for sensor innacuracy
    // compare the computed angle from sensor's x-coordinates with the desired angle
    if (a - ((abs(total_x) * (float)180) / ((float)135 * PI)) < 1) {
      stopRover();
      if (!UARTdataSent) {
        // send the angle back to control: need to remove the offset from the measured angle
        angle = -(((abs(total_x) * (float)180) / ((float)135 * PI)) - ((d_a * (float)20) / (float)360) + (float)1 );  // angle is negative beacause it is counterclockwise
        theta += angle; //the angle should be added to the previous angle in order to compute the current coordinates
        Serial1.print('a');
        Serial1.println(String(theta));
        Serial1.println('d');
        UARTdataSent = true;
      }
      command = '\0';
      d = 0;
      total_x1 = 0;
      total_y1 = 0;
      distance_x = 0;
      distance_y = 0;
    } else {
      // if the desired angle does not the match computed angle, then move turn counterclockwise
      DIRRstate = LOW;
      DIRLstate = HIGH;
      UARTdataSent = false;
    }
  }
  //moving backwards
  else if (command == 'b') {
    // initialize speed when it moves backwards
    digitalWrite(pwmr, HIGH);   //setting right motor speed at maximum
    digitalWrite(pwml, HIGH);   //setting left motor speed at maximum
    int y = d_a;
    // compare sensor's y-coordinates with the desired distance
    if (y - abs(total_y) < 2) {
      stopRover();
      L = -(abs(total_y));
      sendCoordinates();
      command = '\0';
      d = 0;
      total_x1 = 0;
      total_y1 = 0;
      distance_x = 0;
      distance_y = 0;
    } else {
      // if y-coordinates do not match with the desired distance, then move backwards
      DIRRstate = HIGH;
      DIRLstate = HIGH;
      UARTdataSent = false;
    }
  }

  else {
    // if no command is received then coordinates of the sensor should be reset and rover stoped
    stopRover();
    total_x1 = 0;
    total_y1 = 0;
    distance_x = 0;
    distance_y = 0;
  }

  digitalWrite(DIRR, DIRRstate);
  digitalWrite(DIRL, DIRLstate);
}


//************************** Stop Rover ***************************//

void stopRover() {
  digitalWrite(pwmr, 0);
  digitalWrite(pwml, 0);
}

void stopRoverOnCommand() {
  if (command == 'x') {
    Serial1.println('d');
    digitalWrite(pwmr, 0);
    digitalWrite(pwml, 0);
    command = '\0';

    L = abs(total_y);
    long distanceTravelled = L;
    sendCoordinatesOnCommand();
    Serial1.println(String(distanceTravelled));
    UARTdataSent = false;
  }
}

// *********************** Coordinates ***************************//

void sendCoordinates() {
  if (!UARTdataSent) {

    Serial.println("L: " + String(L));
    Serial.println("angle: " + String(angle));
    Serial.println("theta: " + String(theta));

    long b_x = L * sin(theta * PI / 180L);  // find last x-point
    long b_y = L * cos(theta * PI / 180L);  // find last y-point

    last_x += b_x;                  // find x-coordinates
    last_y += b_y;                  // find y-coordinates

    Serial.println("x-coordinates: " + String(last_x));
    Serial.println("y-coordinates: " + String(last_y));

    String coordinatesToSend = "x" + String(last_x) + "y" + String(last_y);
    Serial1.println(coordinatesToSend);
    Serial1.print('a');
    Serial1.println(String(theta));
    Serial1.println('d');
    UARTdataSent = true;          // set flag to true so loop will not execute again 

  }
}

void sendCoordinatesOnCommand() {
  if (!UARTdataSent) {

    // theta += angle;                                // increment theta with angle
    Serial.println("L2: " + String(L));
    Serial.println("angle: " + String(angle));
    Serial.println("theta: " + String(theta));

    long b_x = L * sin(theta * PI / 180L);  // find last x-point
    long b_y = L * cos(theta * PI / 180L);  // find last y-point

    last_x += b_x;                  // find x-coordinates
    last_y += b_y;                  // find y-coordinates

    Serial.println("x-coordinates: " + String(last_x));
    Serial.println("y-coordinates: " + String(last_y));

    String coordinatesToSend = "x" + String(last_x) + "y" + String(last_y);
    Serial1.println(coordinatesToSend);
    Serial1.print('a');
    Serial1.println(String(theta));
    Serial1.print('t');
    UARTdataSent = true;

  }
}


// Timer A CMP1 interrupt. Every 800us the program enters this interrupt.
// This, clears the incoming interrupt flag and triggers the main loop.

ISR(TCA0_CMP1_vect) {
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
  loopTrigger = 1;
}

// This subroutine processes all of the analogue samples, creating the required values for the main loop

void sampling() {

  // Make the initial sampling operations for the circuit measurements

  sensorValue0 = analogRead(A0); //sample Vb

  //**************** VELOCITY CONTROL - Vref ********************//

  if (command == 's') {
    if (d == 10) {
      sensorValue2 = 1023;
      Serial1.println('d');
      command = '\0';
      d = 0;
    }
    else if (d == 9) {
      sensorValue2 = 980;
      Serial1.println('d');
      command = '\0';
      d = 0;
    }
    else if (d == 8) {
      sensorValue2 = 940;
      Serial1.println('d');
      command = '\0';
      d = 0;
    }
    else if (d == 7) {
      sensorValue2 = 890;
      Serial1.println('d');
      command = '\0';
      d = 0;
    }
    else if (d == 6) {
      sensorValue2 = 840;
      Serial1.println('d');
      command = '\0';
      d = 0;
    }
    else if (d == 5) {
      sensorValue2 = 780;
      Serial1.println('d');
      command = '\0';
      d = 0;
    }
    else if (d == 4) {
      sensorValue2 = 730;
      Serial1.println('d');
      command = '\0';
      d = 0;
    }
    else if (d == 3) {
      sensorValue2 = 670;
      Serial1.println('d');
      command = '\0';
      d = 0;
    }
    else if (d == 2) {
      sensorValue2 = 600;
      Serial1.println('d');
      command = '\0';
      d = 0;
    }
    else if (d == 1) {
      sensorValue2 = 500;
      Serial1.println('d');
      command = '\0';
      d = 0;
    }
  }

  //Serial.println(sensorValue2);

  sensorValue3 = analogRead(A3); //sample Vpd
  current_mA = ina219.getCurrent_mA(); // sample the inductor current (via the sensor chip)

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023
  // representing a voltage between 0 and the analogue reference which is 4.096V

  vb = sensorValue0 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts
  vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
  vpd = sensorValue3 * (4.096 / 1023.0); // Convert the Vpd sensor reading to volts

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages

  if (Boost_mode == 1) {
    iL = -current_mA / 1000.0;
    dutyref = saturation(sensorValue2 * (1.0 / 1023.0), 0.99, 0.33);
  } else {
    iL = current_mA / 1000.0;
    dutyref = sensorValue2 * (1.0 / 1023.0);
  }


  // send current to energy
  long I_in = (current_mA * vb * 330) / (vpd * 890);
  //Serial1.println('i');
  //Serial1.println("Current: "+String(I_in));
}

float saturation( float sat_input, float uplim, float lowlim) { // Saturatio function
  if (sat_input > uplim) sat_input = uplim;
  else if (sat_input < lowlim ) sat_input = lowlim;
  else;
  return sat_input;
}

void pwm_modulate(float pwm_input) {           // PWM function
  analogWrite(6, (int)(255 - pwm_input * 255));
}

// This is a PID controller for the voltage

float pidv( float pid_input) {
  float e_integration;
  e0v = pid_input;
  e_integration = e0v;

  //anti-windup, if last-time pid output reaches the limitation, this time there won't be any intergrations.
  if (u1v >= uv_max) {
    e_integration = 0;
  } else if (u1v <= uv_min) {
    e_integration = 0;
  }

  delta_uv = kpv * (e0v - e1v) + kiv * Ts * e_integration + kdv / Ts * (e0v - 2 * e1v + e2v); //incremental PID programming avoids integrations.there is another PID program called positional PID.
  u0v = u1v + delta_uv;  //this time's control output

  //output limitation
  saturation(u0v, uv_max, uv_min);

  u1v = u0v; //update last time's control output
  e2v = e1v; //update last last time's error
  e1v = e0v; // update last time's error
  return u0v;
}

// This is a PID controller for the current

float pidi(float pid_input) {
  float e_integration;
  e0i = pid_input;
  e_integration = e0i;

  //anti-windup
  if (u1i >= ui_max) {
    e_integration = 0;
  } else if (u1i <= ui_min) {
    e_integration = 0;
  }

  delta_ui = kpi * (e0i - e1i) + kii * Ts * e_integration + kdi / Ts * (e0i - 2 * e1i + e2i); //incremental PID programming avoids integrations.
  u0i = u1i + delta_ui;  //this time's control output

  //output limitation
  saturation(u0i, ui_max, ui_min);

  u1i = u0i; //update last time's control output
  e2i = e1i; //update last last time's error
  e1i = e0i; // update last time's error
  return u0i;
}


/*end of the program.*/
