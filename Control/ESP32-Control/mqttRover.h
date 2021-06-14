#ifndef mqttrover_h
#define mqttrover_h

#include <cassert>
#include <PubSubClient.h>               //MQTT Client library

#include "wifiRover.h"
#include "instructionQueue.h"

#include "flags.h"                      //for immediate 'ST' instruction support
#include "coordinateInstructions.h"     //for 'RS' instruction support


// -------- MQTT BROKER AND TOPICS --------

const char *mqttServer = "3.8.124.71";
int mqttPort = 1883;

const char *mqttInTopicInstruction = "command";
const char *mqttOutTopicPosition = "rover";
const char *mqttOutTopicObstacle = "obstacle";
const char *mqttOutTopicBattery = "battery";

//debug
const char *mqttDebugTopic = "debug";

//defines the length of the instruction identifier for when receiving instructions
const int commandSize = 2;



// ----------- MQTT FUNCTIONS -----------

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//Handles received messages on topics that the client is subscribed to
void callback(char* topic, byte* payload, unsigned int length) {
  assert(length >= commandSize);
  Serial.print("info (from Command)\t: received command \'");
  byte c_buffer[commandSize];
  byte v_buffer[length - commandSize];
  for (int i = 0; i < length; i++) {
    if (i < commandSize) {
      c_buffer[i] = payload[i];
    } else {
      v_buffer[i - commandSize] = payload[i];
    }
  }
  c_buffer[commandSize] = '\0';
  v_buffer[length - commandSize] = '\0';
  String receivedCommand = (char*)c_buffer;
  String receivedValue = (char*)v_buffer;
  receivedCommand.trim();
  receivedValue.trim();
  Serial.print(receivedCommand);
  Serial.print("\' \'");
  Serial.print(receivedValue);
  Serial.println("\'");

  //add the instruction to queue of instruction (or execute directly if Stop or Reset instr.)
  if (receivedCommand == "ST") {
    //Stop the rover directly if we receive stop instruction from Command
    Serial1.print('x');
    driveWaiting = false;
  } else if (receivedCommand == "RS") {
    //reset internal variables
    Serial.println("debug (from Command)\t: resetting internal variables");
    theta = 0;
    last_x = 0;
    last_y = 0;
  } else {
    Instruction currentInstr = {receivedCommand, receivedValue};
    instructionQueue.push(currentInstr);
  }
}

//Sets what IP address and what port to connect to
void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  Serial.print("Looking up MQTT Broker at ");
  Serial.print(mqttServer);
  Serial.print(" on port ");
  Serial.print(mqttPort);
  Serial.println(".");
  // set the callback function
  mqttClient.setCallback(callback);
}

//Connects/reconnects to the MQTT Broker server
void reconnect() {
  Serial.print("Connecting to Broker");

  while (!mqttClient.connected()) {
    Serial.print("...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    mqttClient.connect(clientId.c_str());
    if (mqttClient.connected()) {
      // subscribe to topics
      mqttClient.subscribe(mqttInTopicInstruction);
      Serial.println("Connected and subscribed.");
    } else {
      // connection failed
      Serial.print(" error while connecting to broker: ");
      Serial.println(mqttClient.state()); //will provide more information on why it failed.
    }
  }
}



// ----------- HELPER FUNCTIONS -----------

//Converts an Arduino String to char*
char* ArduinoStringToChar(String inputString) {
  if (inputString.length() != 0) {
    char *charArray = const_cast<char*>(inputString.c_str());
    return charArray;
  }
}


#endif
