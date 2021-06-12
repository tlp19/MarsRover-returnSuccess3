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
unsigned int rest_timer, V_Counter,Discharge_Counter, Relay_Counter;
unsigned int loop_trigger;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.
unsigned int upperindex1, lowerindex1,upperindex2, lowerindex2; //upper and lower index for the OCV-SoC graph
float ev=0,cv=0,ei=0,oc=0; //internal signals
float kpv=0.05024,kiv=15.78,kdv=0; // voltage pid.
float u0v,u1v,delta_uv,e0v,e1v,e2v; // u->output; e->error; 0->this time; 1->last time; 2->last last time
float u0i, u1i, delta_ui, e0i, e1i, e2i; // Internal values for the current controller
float uv_max=4, uv_min=0; //anti-windup limitation
float ui_max = 1, ui_min = 0; //anti-windup limitation
float kpi = 0.02512, kii = 39.4, kdi = 0; // current pid.
float Ts = 0.001; //1 kHz control frequency.
float current_lim,current_measure,previous_current,current_ref = 0, error_amps; // Current Control
float pwm_out;
float V_Bat,Vref;
float SoC=100; //State of Charge
float Area; //Area of Current over time using trapezoid method
float Capacitance1_Max = 569.861,Capacitance2_Max = 586.5, Capacitance_Consumed;
float Current_Counter = 0;
float OCV_1,OCV_2; //Open Circuit Voltage for cell 1 and cell 2
float Constraint = 100;
float upperpoint1, lowerpoint1; //upper and lower OCV with the upper and lower indexes
float SoC1_Upper, SoC1_Lower, SoC1_Approx; //Approximation of SoC with function
float upperpoint2, lowerpoint2; //upper and lower OCV with the upper and lower indexes
float SoC2_Upper, SoC2_Lower, SoC2_Approx; //Approximation of SoC with function
boolean input_switch;
boolean relay_switch;
int state_num=0,next_state;
String dataString;

void setup() {
  //Some General Setup Stuff

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

  //LEDs on pin 7 and 8
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  //Relay and Measure pins
  pinMode(A1,INPUT); //Measure Pin for cell 1
  pinMode(4,OUTPUT); //Relay Pin for cell 1

  pinMode(A6,INPUT); //Measure Pin for cell 2
  pinMode(5,OUTPUT); //Relay Pin for cell 2

  //Analogue input, the battery voltage (also port B voltage)
  pinMode(A0, INPUT);

  // TimerA0 initialization for 1kHz control-loop interrupt.
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm;

  // TimerB0 initialization for PWM output
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz

  interrupts();  //enable interrupts.
  analogWrite(6, 120); //just a default state to start with

}

void loop() {
  if (loop_trigger == 1){ // FAST LOOP (1kHZ)
      state_num = next_state; //state transition
      V_Bat = analogRead(A0)*4.096/1.03; //check the battery voltage (1.03 is a correction for measurement error, you need to check this works for you)

      if(relay_switch != 1){
        if ((V_Bat > 3700 || V_Bat < 2400)) { //Checking for Error states (just battery voltage for now)
          state_num = 7; //go directly to jail
          next_state = 7; // stay in jail
          digitalWrite(7,true); //turn on the red LED
          current_ref = 0; // no current
        }
      }
            
    current_measure = (ina219.getCurrent_mA());
    float iL = current_measure/1000;
    error_amps = (current_ref - current_measure) / 1000;

    if(state_num == 2){
        //Initialise Array
        float OCV1_Array[] = {3427.91,3332.47,3332.47,3336.45,3332.47,3328.5,3320.54,3308.61,3300.66,3300.66,3296.68,3296.68,3292.71,3288.73,3276.8,3264.87,3248.96,3225.1,3209.2,3125.69}; // 20 elements
        float OCV2_Array[] = {3463.71,3372.24,3372.20,3372.16,3372.12,3368.26,3364.29,3352.36,3340.43,3336.45,3336.45,3332.47,3332.47,3324.52,3316.57,3304.64,3288.73,3268.85,3252.94,3217.15,2962.64}; // 21 elements
        int SoC_Array[] = {100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5};

        for(int i=0; i<20; i++){
          if(OCV_1 > OCV1_Array[i]){
          upperindex1 = i;
          lowerindex1 = i-1; 
          break;
        }
      }

      for(int i=0; i<21; i++){
          if(OCV_2 > OCV2_Array[i]){
          upperindex2 = i;
          lowerindex2 = i-1; 
          break;
        }
      }
        
        upperpoint1 = OCV1_Array[upperindex1];
        lowerpoint1 = OCV1_Array[lowerindex1];
        SoC1_Upper = SoC_Array[upperindex1];
        SoC1_Lower = SoC_Array[lowerindex1];

        upperpoint2 = OCV2_Array[upperindex2];
        lowerpoint2 = OCV2_Array[lowerindex2];
        SoC2_Upper = SoC_Array[upperindex2];
        SoC2_Lower = SoC_Array[lowerindex2];
        
        if((upperpoint1 - lowerpoint1) == 0){
          SoC1_Approx = (SoC1_Upper + SoC1_Lower)/2;
        }else{
          SoC1_Approx = InitialSoC(upperindex1,SoC1_Upper,OCV_1,upperpoint1, lowerpoint1);
        }

        if((upperpoint2 - lowerpoint2) == 0){
          SoC2_Approx = (SoC2_Upper + SoC2_Lower)/2;
        }else{
          SoC2_Approx = InitialSoC(upperindex2,SoC2_Upper,OCV_2,upperpoint2, lowerpoint2);
        }
        
    }
      
    if(state_num == 4){
      ev = (Vref - V_Bat)/1000;  //voltage error at this time
      cv=pidv(ev);  //voltage pid
      cv=saturation(cv, current_lim, 0); //current demand saturation
      if(V_Counter > 1){
      error_amps=cv - iL; //current error
      }
    }
    
      pwm_out = pidi(error_amps); //Perform the PID controller calculation
      pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
      analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
      int_count++; //count how many interrupts since this was last reset to zero
      loop_trigger = 0; //reset the trigger and move on with life
  
 }
  
  if (int_count == 1000) { // SLOW LOOP (1Hz)
    input_switch = digitalRead(2); //get the OL/CL switch status
    switch (state_num) { // STATE MACHINE (see diagram)
      case 0:{ // Start state (no current, no LEDs)
        current_ref = 0;
        if (input_switch == 1) { // if switch, move to charge
          next_state = 1;
          digitalWrite(8,true);
        } else { // otherwise stay put
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      case 1:{ //Relay is high, measure the OCV, NEED TO REMOVE RESTRICTION ON V BETWEEN 3700 AND 2400
        current_ref = 0;
        if(Relay_Counter < 250){
          digitalWrite(4, true);
          digitalWrite(5,true);
          next_state = 1;
          OCV_1 = analogRead(A1)*4.096/1.03;
          OCV_2 = analogRead(A6)*4.096/1.03;
          relay_switch = 1; 
          digitalWrite(8,true);
          Relay_Counter++;
        }
        else{
          digitalWrite(4,false);
          digitalWrite(5,false);
          next_state = 2;
          relay_switch = 1;
          digitalWrite(8,false);
          Relay_Counter = 0;
        }
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
          relay_switch = 0;
          Relay_Counter = 0;
        }
        break;
      }
      case 2:{
        current_ref = 0;
        next_state = 2;
        digitalWrite(8,false);
        if(input_switch == 0){
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }

      case 3: { // ERROR state RED led and no current
        current_ref = 0;
        next_state = 3; // Always stay here
        digitalWrite(7,true);
        digitalWrite(8,false);
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(7,false);
        }
        break;
      }

      default :{ // Should not end up here ....
        Serial.println("Boop");
        current_ref = 0;
        next_state = 3; // So if we are here, we go to error
        digitalWrite(7,true);
      }
    }
    
    dataString = String(state_num) + "," + String(V_Bat) + "," + String(current_ref) + "," + String(current_measure) + "," + String(OCV_1)+","+String(OCV_2)+","+String(Relay_Counter) +"," +String(upperindex1) +","+ String(lowerindex1)+"," +String(upperpoint1) +","+ String(lowerpoint1)+","+ String(SoC1_Upper)+","+String(SoC1_Lower)+"," +String(upperindex2) +","+ String(lowerindex2)+"," +String(upperpoint2) +","+ String(lowerpoint2)+","+ String(SoC2_Upper)+","+String(SoC2_Lower)+","+String(SoC1_Approx)+","+String(SoC2_Approx); //build a datastring for the CSV file
    Serial.println(dataString); // send it to serial as well in case a computer is connected
    File dataFile = SD.open("BatCycle.csv", FILE_WRITE); // open our CSV file
    if (dataFile){ //If we succeeded (usually this fails if the SD card is out)
      dataFile.println(dataString); // print the data
    } else {
      Serial.println("File not open"); //otherwise print an error
    }
    dataFile.close(); // close the file
    int_count = 0; // reset the interrupt count so we dont come back here for 1000ms
   
  }
}

// Timer A CMP1 interrupt. Every 1000us the program enters this interrupt. This is the fast 1kHz loop
ISR(TCA0_CMP1_vect) {
  loop_trigger = 1; //trigger the loop when we are back in normal flow
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
}

float saturation( float sat_input, float uplim, float lowlim){ // Saturation function
  if (sat_input > uplim) sat_input = uplim;
  else if (sat_input < lowlim ) sat_input = lowlim;
  else;
  return sat_input;
}

//Return SoC value for given OCV
//index is array index of point to get equation of line
//upperpoint OCV and lowerpoint OCV
//Returns SoC of the battery

float InitialSoC(int upperindex,int SoC_Upper, float OCV, float upperpoint, float lowerpoint){

    float gradient = (lowerpoint-upperpoint)/5;
    float c = upperpoint - gradient*SoC_Upper;
    float SoC_approximation = (OCV - c)/gradient;
    return SoC_approximation;
}

float pidv( float pid_input){
  float e_integration;
  e0v = pid_input;
  e_integration = e0v;
 
  //anti-windup, if last-time pid output reaches the limitation, this time there won't be any intergrations.
  if(u1v >= uv_max) {
    e_integration = 0;
  } else if (u1v <= uv_min) {
    e_integration = 0;
  }

  delta_uv = kpv*(e0v-e1v) + kiv*Ts*e_integration + kdv/Ts*(e0v-2*e1v+e2v); //incremental PID programming avoids integrations.there is another PID program called positional PID.
  u0v = u1v + delta_uv;  //this time's control output

  //output limitation
  saturation(u0v,uv_max,uv_min);
  
  u1v = u0v; //update last time's control output
  e2v = e1v; //update last last time's error
  e1v = e0v; // update last time's error
  return u0v;
}

float pidi(float pid_input) { // discrete PID function
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
