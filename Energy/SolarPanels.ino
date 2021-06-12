#include <Wire.h>
#include <INA219_WE.h>
#include <SPI.h>
#include <SD.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = 10;
unsigned int loop_trigger;
float Ts = 0.001; //1 kHz control frequency.
unsigned int sensor_Vb,sensor_Vref,sensor_Vpd;  // ADC sample values declaration
float vb,vref,vpd,va;
float current_limit,oc,iL,current_mA; // Current Control
float power_out;
float open_loop,dutyref;
boolean input_switch;
String dataString;


void setup() {
  
  Wire.begin(); // We need this for the i2c comms for the current sensor
  Wire.setClock(700000); // set the comms speed for i2c
  ina219.init(); // this initiates the current sensor
  Serial.begin(9600); // USB Communications


  //Check for the SD Card
  Serial.println("\nInitializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("* is a card inserted?");
    while (true) {} //It will stick here FOREVER if no SD is in on boot
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  if (SD.exists("BatCycle.csv")) { // Wipe the datalog when starting
    SD.remove("BatCycle.csv");
  }

  noInterrupts(); //disable all interrupts
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  //SMPS Pins
  pinMode(13, OUTPUT); // Using the LED on Pin D13 to indicate status
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  pinMode(6, OUTPUT); // This is the PWM Pin

  //Port A and Port B pins
  pinMode(A0,INPUT); //Port B
  pinMode(A2,INPUT); //Port A
  pinMode(A3,INPUT); // Vref

  // TimerA0 initialization for control-loop interrupt.
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm; 

  TCB0.CTRLA=TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz

  interrupts();  //enable interrupts.
  analogWrite(6, 120); //just a default state to start with

}

void loop() {
  if(loop_trigger){

    sampling();
    input_switch = digitalRead(2);

    if(input_switch != 1){
    current_limit = 0.46; // 2 panels in parallel have a current limit of 0.46A
    oc = iL-current_limit; // Calculate the difference between current measurement and current limit
     if ( oc > 0) {
     open_loop=open_loop-0.001; // We are above the current limit so less duty cycle
     } else {
     open_loop=open_loop+0.001; // We are below the current limit so more duty cycle
     }
     open_loop=saturation(open_loop,dutyref,0.02); // saturate the duty cycle at the reference or a min of 0.01
     pwm_modulate(open_loop); // and send it out
    }

    dataString = String(vpd) + "," + String(vref) + "," + String(vb) + "," + String(va) + "," + String(current_mA) +","+String(power_out)+ "," + String(dutyref) + "," +String(open_loop); //build a datastring for the CSV file
    Serial.println(dataString); // send it to serial as well in case a computer is connected
    File dataFile = SD.open("BatCycle.csv", FILE_WRITE); // open our CSV file
    if (dataFile){ //If we succeeded (usually this fails if the SD card is out)
      dataFile.println(dataString); // print the data
    } else {
      Serial.println("File not open"); //otherwise print an error
    }
    dataFile.close(); // close the file
  }
}

ISR(TCA0_CMP1_vect) {
  loop_trigger = 1; //trigger the loop when we are back in normal flow
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
}

void sampling(){

  // Make the initial sampling operations for the circuit measurements
  
  sensor_Vb = analogRead(A0); //sample Vb
  sensor_Vpd = analogRead(A2); //sample Vpd
  sensor_Vref = analogRead(A3); //sample Vref(Control knob)
  current_mA = ina219.getCurrent_mA(); // current at port B since the current measuring resistor is present here

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  vb = sensor_Vb * (4.096 / 1.03); // Convert the Vb sensor reading to volts
  vref = sensor_Vref * (4.096 / 1.03); // Convert the Vref sensor reading to volts
  vpd = sensor_Vpd * (4.096 / 1.03); // Convert the Vpd sensor reading to volts
  va = vpd * (float)(890/330);

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages

    power_out = (float)(current_mA * vb);
    iL = current_mA/1000.0;
    dutyref = vref * (1.0 / 4000);
}

float saturation( float sat_input, float uplim, float lowlim){ // Saturatio function
  if (sat_input > uplim) sat_input=uplim;
  else if (sat_input < lowlim ) sat_input=lowlim;
  else;
  return sat_input;
}

float MPPT(float V_in,float I_in,){

  float Power_in = V_in * I_in;
  
}

void pwm_modulate(float pwm_input){ // PWM function
  analogWrite(6,(int)(255-pwm_input*255)); 
}
