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
unsigned int rest_timer, V_Counter,Discharge_Counter, Relay_Counter,SoC_ChargeCounter,Rest_Counter;
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
float Capacitance_Max = 569.861 , Capacitance_Consumed; //Capacitance2_Max = 586.5;
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
  // put your main code here, to run repeatedly:
  if(loop_trigger == 1){
    state_num = next_state; //state transition
      V_Bat = analogRead(A0)*4.096/1.03; //check the battery voltage (1.03 is a correction for measurement error, you need to check this works for you)
            
    current_measure = (ina219.getCurrent_mA());
    float iL = current_measure/1000;
    error_amps = (current_ref - current_measure) / 1000;
    int_count++; //count how many interrupts since this was last reset to zero
    loop_trigger = 0; //reset the trigger and move on with life
  }

  if(int_count == 1000){
    input_switch = digitalRead(2); //get the OL/CL switch status
    switch (state_num) { // STATE MACHINE (see diagram)
      case 0:{
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
      case 1:{ // Start state (no current, no LEDs)
        current_ref = 0;
        SoC = SoC - 0.02;
        Serial.println(SoC);
        next_state = 1;
        break;
      }
      case 2:{ // error state 
        current_ref = 0;
        next_state = 2; // Always stay here
        digitalWrite(7,true);
        digitalWrite(8,false);
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(7,false);
        }
        break;
      }
  }
    int_count = 0; // reset the interrupt count so we dont come back here for 1000ms
  }
}

// Timer A CMP1 interrupt. Every 1000us the program enters this interrupt. This is the fast 1kHz loop
ISR(TCA0_CMP1_vect) {
  loop_trigger = 1; //trigger the loop when we are back in normal flow
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
}
