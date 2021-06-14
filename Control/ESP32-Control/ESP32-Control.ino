#include "flags.h"
#include "wifiRover.h"
#include "mqttRover.h"
#include "instruction.h"
#include "instructionQueue.h"

#include <map>


//Pins for Drive UART connection (Serial1)
#define RXD2 16         //Pin 9 on adapter PCB - connected to arduino TX
#define TXD2 17         //Pin 8 on adapter PCB - connected to arduino RX

//Pins for Vision UART connection (Serial2)
#define RXD3 22         //Pin 3 on adapter PCB
#define TXD3 23         //Pin 2 on adapter PCB



// ----------------- DRIVE & VISION VARIABLES ------------------


//Drive variables
long distanceTravelled;                     //if instruction is interrupted, gives the distance already travelled
long roverAngle;                            //to find the coordinates of an obstacle
long xRoverCoordinate, yRoverCoordinate;    //to find the coordinates of an obstacle

//Vision variables
std::map<String, String> obstacleList;      //list of current obstacles detected by vision
String obstacleColorToAvoid;                //color of the obstacle to avoid



// ---------------- DRIVE UART FUNCTIONS ----------------


//Send an instruction through the Drive UART port
void sendInstructionDriveUART(const Instruction &toSend) {
  driveWaiting = false;
  String sendInstruction = decodeInstr(toSend);
  Serial.print("info (to Drive)\t\t: sending through Drive UART: ");
  Serial.println(sendInstruction);
  Serial1.print(sendInstruction);

  //debug
  String debugMsg = "DriveUART: sending " + sendInstruction;
  mqttClient.publish(mqttDebugTopic, ArduinoStringToChar(debugMsg));
}

//Receive and decode data from the Drive UART connection
void receiveDataDriveUART() {
  char fromDrive;
  //For when reading coordinates
  bool readingX = false;
  bool readingY = false;
  String xCoordinate = "";
  String yCoordinate = "";
  //For when reading distance already travelled
  bool readingTravelled = false;
  String readTravelled = "";
  //For when reading absolute angle of rover
  bool readingAngle = false;
  String readAngle = "";

  while (Serial1.available()) {
    fromDrive = Serial1.read();
    if ((fromDrive != '\r') && (fromDrive != '\n')) {
      Serial.print("debug (from Drive)\t: received from Drive UART: ");
      Serial.println(fromDrive);
      if (fromDrive == 'd') {           //rover is done with last instruction
        driveWaiting = true;
        Serial.println("info (from Drive)\t: rover is ready for next instruction");
        mqttClient.publish(mqttDebugTopic, "DriveUART: done with instr.");
      } else if (fromDrive == 'x') {    //receiving x coordinates of rover
        readingX = true;
      } else if (fromDrive == 'y') {    //receiving y coordinates of rover
        readingX = false;
        readingY = true;
      } else if (fromDrive == 't') {    //receiving distance travelled by rover since last instr.
        readingTravelled = true;
      } else if (fromDrive == 'a') {    //receiving the orientation of rover
        readingAngle = true;
      } else {
        if (readingX) {
          xCoordinate += fromDrive;
        } else if (readingY) {
          yCoordinate += fromDrive;
        } else if (readingTravelled) {
          readTravelled += fromDrive;
        } else if (readingAngle) {
          readAngle += fromDrive;
        } else {
          mqttClient.publish(mqttDebugTopic, "DriveUART: error (else)");
        }
      }
    } else if (fromDrive == '\r') {
      if (readingY) {
        readingX = false;
        readingY = false;
        //clean up the strings read
        xCoordinate.trim();
        yCoordinate.trim();
        //update the coordinates of the rover stored internally
        xRoverCoordinate = xCoordinate.toInt();
        last_x = xRoverCoordinate;
        yRoverCoordinate = yCoordinate.toInt();
        last_y = yRoverCoordinate;
        //send coordinates to Command
        String fullCoordinates = xCoordinate + ":" + yCoordinate;
        mqttClient.publish(mqttOutTopicPosition, ArduinoStringToChar(fullCoordinates));

        Serial.print("info (from Drive)\t: coordinates of rover are: ");
        Serial.println(fullCoordinates);

      } else if (readingTravelled) {
        readingTravelled = false;
        readTravelled.trim();
        distanceTravelled = readTravelled.toInt();
      } else if (readingAngle) {
        readingAngle = false;
        readAngle.trim();
        roverAngle = readAngle.toInt();
        theta = roverAngle;
        Serial.print("info (from Drive)\t: orientation of rover is: ");
        Serial.println(roverAngle);
      }
    }
    delay(10); //avoid collision of data due to processing speed differences
  }
  fromDrive = '\0';
}



// ---------------- VISION UART FUNCTIONS ----------------


//Receive and decode data from the Vision UART connection
void receiveDataVisionUART() {
  char fromVision = '\0';
  //For when reading color of obstacle
  bool readingColor = false;
  String readColor = "";
  //For when reading distance of obstacle
  bool readingDistance = false;
  String readDistance = "";

  while (Serial2.available() && (fromVision != 'e')) {
    fromVision = Serial2.read();
    if ((fromVision != '\r') && (fromVision != '\n')) {
      if (fromVision == 'b') {          //beginning of current transmission
        //reset map at beginning
        obstacleList.clear();
      } else if (fromVision == 'c') {   //receiving color of detected obstacle
        readingColor = true;
      } else if (fromVision == 'd') {   //receiving distance of detected obstacle
        readingColor = false;
        readingDistance = true;
      } else {
        if (readingColor) {
          readColor += fromVision;
        } else if (readingDistance) {
          readDistance += fromVision;
        }
      }
    } else if (fromVision == '\r') {
      if (readingDistance) {
        readingDistance = false;
        //clean up the strings read
        readColor.trim();
        readDistance.trim();
        //add the obstacle (color:distance) to the list of detected obstacles
        obstacleList[readColor] = readDistance;
        //reset strings for next obstacle to be read
        readColor = "";
        readDistance = "";
      }
    }
    delay(15); //avoid collision of data due to processing speed differences
  }
  fromVision = '\0';
}



// ---------------- DRIVE-VISION-COMMAND FUNCTIONS ----------------


//Computes the coordinates of the obstacle that triggered the obstacle avoidance protocol, and sends them to Command
void computeAndSendObstacleCoordinatesToCommand(String obstacleColor) {
  long distanceToObstacle = lengthOfRover + (obstacleDetectionThreshold * 3 / 4);
  long xObstacleCoordinate = xRoverCoordinate + ((distanceToObstacle) * sin(roverAngle * PI / 180L));
  long yObstacleCoordinate = yRoverCoordinate + ((distanceToObstacle) * cos(roverAngle * PI / 180L));
  String obstacleCoordinates = String(xObstacleCoordinate) + ":" + String(yObstacleCoordinate);
  Serial.print("info (internal)\t\t: coordinates of obstacle are: ");
  Serial.println(obstacleCoordinates);
  String mqttSendObstacle = obstacleColor + obstacleCoordinates;
  mqttClient.publish(mqttOutTopicObstacle, ArduinoStringToChar(mqttSendObstacle));
}





// ---------------------- MAIN CODE ----------------------


//Timers for scheduling of functions in loop() function
unsigned long lastTimer = 0;   //5s loop

//debug
bool debugLogDoneOnce = false;


void setup() {

  //Setup serial communications
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);      //Control-Drive UART
  Serial2.begin(9600, SERIAL_8N1, RXD3, TXD3);      //Control-Vision UART
  Serial.println();

  //Setup wireless connections
  connectToWiFi();
  setupMQTT();
  reconnect();

  //Initialize flag variables
  //for Drive:
  driveWaiting = true;        //Drive is ready to receive on start-up
  distanceTravelled = -1;     //Initialized at -1 as it is also used as a flag to check if we have received it
  roverAngle = 0;             //on startup, the rover's orientation is denoted as angle 0
  xRoverCoordinate = 0;       //on startup, the coordinates of the rover are denoted as 0:0
  yRoverCoordinate = 0;
  //for Vision:
  obstacleColorToAvoid = "";
  visionOverride = false;     //first set to false to start in normal instr. queue
  playingRoutine = false;     //not playing obstacle avoidance routine on start-up of rover
  currentAvoidRoutine = avoidRoutine; //initialize obstacle avoidance routine

  //Outputs info on obstacle avoidance routine
  Serial.print("Obstacle avoidance routine is set to: ");
  while (currentAvoidRoutine.size() > 0) {
    Serial.print(currentAvoidRoutine.front().command);
    Serial.print(currentAvoidRoutine.front().value);
    Serial.print(" ");
    currentAvoidRoutine.pop();
  }
  Serial.println();
  currentAvoidRoutine = avoidRoutine;

  //debug
  mqttClient.publish(mqttDebugTopic, "--- esp32 connected ---");
}





void loop() {

  unsigned long currentTimer = millis();

  //Reconnect to MQTT Broker if disconnected, and mqtt client loop actions
  if (!mqttClient.connected())
    reconnect();
  mqttClient.loop();

  /*
    //Publish battery status to MQTT topic 'battery/status' regularly
    if ((currentTimer - lastTimer) >= 1000) {
      lastTimer = currentTimer; //loop is being executed, so last execution is now
      //check the battery status from Energy, and publish for Command to fetch
      //mqttClient.publish(mqttOutTopicBattery, "100%"); //note: as connection is so physically established, we use abitrary data
    }
  */
/*
  //Read incoming data from Vision (if any)
  receiveDataVisionUART();
  //Check if we have an obstacle
  if (!obstacleList.empty()) {
    obstaclesDetected = true;
    //iterate through all obstacles detected to see if one is in range
    bool oneBallClose = false;
    for (auto i = obstacleList.begin(); i != obstacleList.end(); i++) {
      Serial.print("[" + i->first + "," + i->second + "] ");
      if (i->second == "C") {
        oneBallClose = true;
        obstacleColorToAvoid = i->first;
      }
    }
    if (oneBallClose) {
      visionOverride = true;
    } else {
      visionOverride = false;
      obstacleColorToAvoid = "";
    }

    //debug
    if (visionOverride == true) {
      Serial.print("OBSTACLE_AVOIDANCE");
      mqttClient.publish(mqttDebugTopic, "CLOSE");
    } else {
      Serial.print("no_obstance_avoidance");
      mqttClient.publish(mqttDebugTopic, "FAR");
    }
    Serial.println();

  } else {
    obstaclesDetected = false;
    visionOverride = false;
    mqttClient.publish(mqttDebugTopic, "nothing detected");
  }
*/
  //Read incoming data from Drive (if any)
  receiveDataDriveUART();

  /*
    //debug: one-time execution block a few seconds into the program to simulate a change in state of the other sub-modules flags
    if ((currentTimer >= 10000) && (debugLogDoneOnce == false)) {
      visionOverride = true;
      mqttClient.publish(mqttDebugTopic, "debug flags");
      Serial.println("debug (internal)\t: simulated flags updated");
      debugLogDoneOnce = true;
    }
  */

  //Handle instructions to give to Drive depending on Vision's input
  if ((visionOverride == false) && (playingRoutine == false)) {

    if (driveWaiting == true) {
      if (instructionQueue.size() > 0) {
        //save the instruction to memory for future obstacle avoidance
        lastInstruction = instructionQueue.front();
        //decode and pass the instruction through UART to Drive
        sendInstructionDriveUART(lastInstruction);
        //delete instruction from top of queue
        if (!instructionQueue.empty()) {
          instructionQueue.pop();
        }
      }
      //no else, if instructionQueue is empty, don't do anything
    }
    //no else, just wait until Drive is finished with prev. instruction


  } else {     // Obstacle has just yet been detected, or we are already in avoidance routine

    if (playingRoutine == false) { //not yet in obstacle avoidance routine

      if (driveWaiting == false) {
        if (lastInstruction.command == "FW") {
          Serial.println("info: Obstacle detected! (case: during forward instruction)");
          mqttClient.publish(mqttDebugTopic, "Obstacle detected! (1)");

          //stop rover & ping Drive for information
          sendInstructionDriveUART(stopInstr);
          //read already travelled distance, coordinates and orientation of rover from Drive
          while (distanceTravelled < 0) {
            receiveDataDriveUART();
          }
          //Send coordinates of detected obstacle to Command
          computeAndSendObstacleCoordinatesToCommand(obstacleColorToAvoid);

          //play routine first
          driveWaiting = true;
          playingRoutine = true;

          //for when routine is finished, resume lastInstruction and normal queue
          std::queue<Instruction> newInstrQueue;
          //find the distance left to travel after having avoided the obstacle
          int distanceLeft = lastInstruction.value.toInt() - distanceTravelled - avoidanceCost;
          //fill new instruction queue with first the instruction to continue (if needed) and then all other instructions in the queue
          if (distanceLeft < 0) {
            //if rover has already exceeded distance to travel, don't resume lastInstruction
          } else {
            //travel the remaining distance of lastInstruction
            Instruction continuedInstruction = {"FW", String(distanceLeft)};
            newInstrQueue.push(continuedInstruction);
          }
          //fill new instruction queue with all previous instructions
          while (!instructionQueue.empty()) {
            newInstrQueue.push(instructionQueue.front());
            instructionQueue.pop();
          }
          instructionQueue = newInstrQueue;
          distanceTravelled = -1;
        }
        //no else, if other instruction, continue normally
      }

      //if driveWaiting AND next intr. is 'fw':
      //note: this could happen if the rover turns and there is an obstacle next to it
      else if (driveWaiting && (!instructionQueue.empty())) {
        if (instructionQueue.front().command == "FW") {
          Serial.println("info: Obstacle detected! (case: upon receiving forward instruction)");
          mqttClient.publish(mqttDebugTopic, "Obstacle detected! (2)");

          //stop rover & ping Drive for information
          sendInstructionDriveUART(stopInstr);
          //read already travelled distance, coordinates and orientation of rover from Drive
          while (distanceTravelled < 0) {
            receiveDataDriveUART();
          }
          distanceTravelled = -1; //here distanceTravelled is only used to check we have received everything

          //Send coordinates of detected obstacle to Command
          computeAndSendObstacleCoordinatesToCommand(obstacleColorToAvoid);

          //play routine to avoid obstacle
          playingRoutine = true;
          driveWaiting = true;

          //susbtract minimum avoidance cost from value of next intr. and add back to top of queue
          int newInstructionValue = instructionQueue.front().value.toInt() - avoidanceCost;
          if (newInstructionValue > 0) {
            instructionQueue.front().value = String(newInstructionValue);
          } else {
            instructionQueue.pop();
          }
        } else {
          //as this not a forward instruction, the rover won't collide with obstacle, so just execute it
          mqttClient.publish(mqttDebugTopic, "Obstacle detected! not FW");

          //save the instruction to memory for future obstacle avoidance
          lastInstruction = instructionQueue.front();
          //decode and pass the instruction through UART to Drive
          sendInstructionDriveUART(lastInstruction);
          //delete instruction from top of queue
          if (!instructionQueue.empty()) {
          instructionQueue.pop();
        }
        }
      }

    } else {    //Obstacle avoidance routine started

      //execute routine, at the end set back flags to low and reinitialize currentAvoidRoutine
      if (driveWaiting == true) {
        if (!currentAvoidRoutine.empty()) {
          //save the instruction to memory for future obstacle avoidance
          lastRoutineInstruction = currentAvoidRoutine.front();
          //decode and pass the instruction through UART to Drive
          sendInstructionDriveUART(lastRoutineInstruction);
          //delete instruction from top of queue
          currentAvoidRoutine.pop();
        } else {
          //routine is finished, so set back flags to normal
          Serial.println("info (internal)\t\t: Obstacle avoidance routine finished");
          playingRoutine = false;
          visionOverride = false;
          currentAvoidRoutine = avoidRoutine;
        }
      } //no else, wait for Drive to be finished with prev. instruction
    }
  }
}
