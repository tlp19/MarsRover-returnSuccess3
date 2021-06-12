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
unsigned int rest_timer, V_Counter, Discharge_Counter;
unsigned int loop_trigger;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.
float ev = 0, cv = 0, ei = 0, oc = 0; //internal signals
float kpv = 0.05024, kiv = 15.78, kdv = 0; // voltage pid.
float u0v, u1v, delta_uv, e0v, e1v, e2v; // u->output; e->error; 0->this time; 1->last time; 2->last last time
float u0i, u1i, delta_ui, e0i, e1i, e2i; // Internal values for the current controller
float uv_max = 4, uv_min = 0; //anti-windup limitation
float ui_max = 1, ui_min = 0; //anti-windup limitation
float kpi = 0.02512, kii = 39.4, kdi = 0; // current pid.
float Ts = 0.001; //1 kHz control frequency.
float current_lim, current_measure, previous_current, current_ref = 0, error_amps; // Current Control
float pwm_out;
float V_Bat, Vref;
float SoC = 100; //State of Charge
float Current_Counter = 0;
float Area; //Area of Current over time using trapezoid method
float Capacitance_Max = 1140, Capacitance_Consumed; //Battery 1 has capacitance_max = 569.861, battery 2 = 586.5
boolean input_switch;
int state_num = 0, next_state;
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
  if (looptrigger == 1) {
    state_num = next_state;
    V_Bat = analogRead(A0) * 4.096 / 1.03; //check the battery voltage (1.03 is a correction for measurement error, you need to check this works for you)
    if ((V_Bat > 3700 || V_Bat < 2400)) { //Checking for Error states (just battery voltage for now)
      state_num = 3; //go directly to jail
      next_state = 3; // stay in jail
      digitalWrite(7, true); //turn on the red LED
      current_ref = 0; // no current
    }
    current_measure = (ina219.getCurrent_mA());
    float iL = current_measure / 1000;
    error_amps = (current_ref - current_measure) / 1000;

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
          Serial.println("100");
          digitalWrite(8,false);
        }
        break;
      }
      case 1: { //Discharge state (-250mA and no LEDs)
          if (V_Bat > 2500) { // While not at minimum volts, stay here
            if (Discharge_Counter > 0) {
              Current_Counter = Current_Counter + current_measure + previous_current;
              Area = (Current_Counter) / 2;
              Capacitance_Consumed = -(Area / 3600) / Capacitance_Max;
              SoC = 100 - (100 * Capacitance_Consumed);
            }
            previous_current = current_measure;
            next_state = 4;
            digitalWrite(8, false);
            Discharge_Counter++;
          } else { // If we reach full discharged, move to rest
            next_state = 5;
            digitalWrite(8, false);
            Discharge_Counter = 0;
          }
          if (input_switch == 0) { //UNLESS the switch = 0, then go back to start
            next_state = 0;
            digitalWrite(8, false);
            Discharge_Counter = 0;
          }
          break;
        }
      case 2: { // Discharge rest, no LEDs no current
          current_ref = 0;
          if (rest_timer < 30) { // Rest here for 30s like before
            next_state = 5;
            digitalWrite(8, false);
            rest_timer++;
          } else { // When thats done, move back to charging (and light the green LED)
            next_state = 1;
            digitalWrite(8, true);
            rest_timer = 0;
          }
          if (input_switch == 0) { //UNLESS the switch = 0, then go back to start
            next_state = 0;
            digitalWrite(8, false);
          }
          break;
        }
      case 3: { // ERROR state RED led and no current
          current_ref = 0;
          next_state = 3; // Always stay here
          digitalWrite(7, true);
          digitalWrite(8, false);
          if (input_switch == 0) { //UNLESS the switch = 0, then go back to start
            next_state = 0;
            digitalWrite(7, false);
          }
          break;
        }

    }
  }
}
