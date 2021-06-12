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
unsigned int rest_timer, V_Counter,Discharge_Counter, Relay_Counter,power_counter;
unsigned int loop_trigger;
unsigned int sensor_Vb,sensor_Vref,sensor_Vpd;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.
float ev=0,cv=0,ei=0,oc=0; //internal signals
float kpv=0.05024,kiv=15.78,kdv=0; // voltage pid.
float u0v,u1v,delta_uv,e0v,e1v,e2v; // u->output; e->error; 0->this time; 1->last time; 2->last last time
float u0i, u1i, delta_ui, e0i, e1i, e2i; // Internal values for the current controller
float uv_max=4, uv_min=0; //anti-windup limitation
float ui_max = 1, ui_min = 0; //anti-windup limitation
float kpi = 0.02512, kii = 39.4, kdi = 0; // current pid.
float Ts = 0.001; //1 kHz control frequency.
float current_lim,iL,OC,current_measure,current_limit,previous_current,current_ref = 0, error_amps; // Current Control
float power_out,previous_power;
float open_loop,dutyref;
float potential_divider = 2.696969696969696969696969696969696969696969696969;
float pwm_out;
float V_Bat,Vref,vb,vref,vpd,va,previous_voltage;
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
  pinMode(A1,INPUT); //Measure Pin
  pinMode(4,OUTPUT); //Relay Pin

  //Analogue input, the battery voltage (also port B voltage)
  pinMode(A0, INPUT); 
  pinMode(A2,INPUT); //Port A
  pinMode(A3,INPUT); // Vref

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
      sampling();

        if(relay_switch != 1){
        if ((V_Bat > 3700 || V_Bat < 2400)) { //Checking for Error states (just battery voltage for now)
          state_num = 6; //go directly to jail
          next_state = 6; // stay in jail
          digitalWrite(7,true); //turn on the red LED
          current_ref = 0; // no current
        }
      }
      
      if(power_counter == 0){
        dutyref = vref * (1.0/4000);
      }
          
    float iL = current_measure/1000;
    error_amps = (current_ref - current_measure) / 1000;

     //For Panel Initialisation
     if(state_num != 2){
     current_limit = 0.46; // 2 panels in parallel have a current limit of 0.46A
     OC = iL-current_limit; // Calculate the difference between current measurement and current limit
     if ( OC > 0) {
     open_loop=open_loop-0.001; // We are above the current limit so less duty cycle
     } else {
     open_loop=open_loop+0.001; // We are below the current limit so more duty cycle
     }
      open_loop=saturation(open_loop,dutyref,0.02); // saturate the duty cycle at the reference or a min of 0.01
      pwm_modulate(open_loop); // and send it out
     }else{
      ev = (Vref - V_Bat)/1000;  //voltage error at this time
      cv=pidv(ev);  //voltage pid
      cv=saturation(cv, current_lim, 0); //current demand saturation
      if(V_Counter > 1){
      error_amps=cv - iL; //current error
      }
      pwm_out = pidi(error_amps); //Perform the PID controller calculation
      pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
      analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
     }
      
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

      case 1:{ //Relay for battery high so we can give the panels enough time to reach maximum power
        if(V_Bat < 3600){
            if(power_counter>0){//after the first cycle, save the values
            dutyref = MPPT(va,previous_voltage,power_out,previous_power,dutyref);
            previous_power = power_out;
            previous_voltage = va;
            power_counter++;
          }else{//first cycle with initial power
            dutyref = MPPT(va,0,power_out,0,dutyref);
            previous_power = power_out;
            previous_voltage = va;
            power_counter++;
          }
          next_state = 1;
          digitalWrite(8,false);
       }else{
        next_state = 2;
        digitalWrite(8,true);
       }
       if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(4,false);
          digitalWrite(8,false);
          Relay_Counter = 0;
        }
       break;
      }
      
      case 2:{ //Constant Max Voltage to obtain higher charge(green LED on and decreasing current till 50mA)     
          Vref = 3600;
          current_lim = 0.25;
          if(current_measure > 50){
            next_state = 2;
            digitalWrite(8,true);
            V_Counter++;
          }
          else{
            next_state = 3;
            digitalWrite(8,false);
            V_Counter = 0;  
          }
          if(input_switch == 0){
            next_state = 0;
            digitalWrite(8,false);
            V_Counter = 0;
          }
          break;
      }
      case 3:{ // Charge Rest, green LED is off and no current
          digitalWrite(4,true);
          next_state = 3;
          digitalWrite(8,false); 
          rest_timer++;
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;        
      }
      case 4: { // ERROR state RED led and no current
        current_ref = 0;
        next_state = 4; // Always stay here
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
        next_state = 4; // So if we are here, we go to error
        digitalWrite(7,true);
      }
    }
    
    dataString = String(state_num) + "," + String(V_Bat)+","+String(current_ref) + "," + String(current_measure)+","+String(previous_power)+","+String(power_out)+","+String(dutyref); //build a datastring for the CSV file
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

void sampling(){

  // Make the initial sampling operations for the circuit measurements
  
  sensor_Vb = analogRead(A0); //sample Vb
  sensor_Vpd = analogRead(A2); //sample Vpd
  sensor_Vref = analogRead(A3); //sample Vref(Control knob)
  current_measure = ina219.getCurrent_mA(); // current at port B since the current measuring resistor is present here

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  vb = sensor_Vb * (4.096 / 1.03); // Convert the Vb sensor reading to volts
  vref = sensor_Vref * (4.096 / 1.03); // Convert the Vref sensor reading to volts
  vpd = sensor_Vpd * (4.096 / 1.03); // Convert the Vpd sensor reading to volts
  va = vpd * potential_divider;

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages

    power_out = current_measure * vb;
}

float saturation( float sat_input, float uplim, float lowlim) { // Saturation function
  if (sat_input > uplim) sat_input = uplim;
  else if (sat_input < lowlim ) sat_input = lowlim;
  else;
  return sat_input;
}

float MPPT(float voltage,float previous_voltage ,float power, float previous_power, float dutyref){

  if((power-previous_power)>0){
    if((voltage-previous_voltage)>0){
      dutyref=dutyref-0.01;
    }else{
      dutyref=dutyref+0.01;
    }
  }else{
     if((voltage-previous_voltage)>0){
      dutyref=dutyref+0.01;
     }else{
      dutyref=dutyref-0.01;
     }
  }
  return dutyref;
  
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

void pwm_modulate(float pwm_input){ // PWM function
  analogWrite(6,(int)(255-pwm_input*255)); 
}
