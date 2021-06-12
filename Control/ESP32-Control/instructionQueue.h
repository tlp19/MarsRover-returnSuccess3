#ifndef instructionqueue_h
#define instructionqueue_h

#include "instruction.h"
#include "coordinateInstructions.h"
#include <queue>


// IMPORTANT: All distances are in millimeters.


// -------- INSTRUCTION QUEUE CONSTANTS --------

//Measurements of real-life objects
//const int sizeOfBall = 50;
const int lengthOfRover = 150;

//Obstacle detection trigger threshold set by Vision
const int obstacleDetectionThreshold = 100;

//Obstacle avoidance path parameters
const String pathLength = String(obstacleDetectionThreshold + lengthOfRover);
const String pathWidth = String(pathLength.toInt() / 2);
const int avoidanceCost = pathLength.toInt();


// -------- INSTRUCTION QUEUE VARIABLES --------


//Normal Instruction Queue
std::queue<Instruction> instructionQueue;
Instruction lastInstruction;

//Stop Instruction
const Instruction stopInstr = {"ST", ""};

//Obstacle Avoidance Routine Queue
const std::deque<Instruction> initQueue{{"CL", "90"}, {"FW", pathWidth}, {"CC", "90"}, {"FW", pathLength}, {"CC", "90"}, {"FW", pathWidth}, {"CL", "90"}};
const std::queue<Instruction> avoidRoutine(initQueue);
std::queue<Instruction> currentAvoidRoutine;
Instruction lastRoutineInstruction;



// -------- INSTRUCTION QUEUE FUNCTIONS --------

//Constructs two instructions based on given coordinates
void insertFrontInstructionQueue(long x, long y) {
  //first find equivalent distance and angle
  translateCoordinates(x, y);
  //new empty instruction queue
  std::queue<Instruction> newInstructionQueue;
  //build turn instruction
  Instruction turnInstruction;
  if (angle != 0) {
    if (angle >= 0) {
      turnInstruction = {"CL", String(angle)};
      newInstructionQueue.push(turnInstruction);
    } else {
      turnInstruction = {"CC", String(-angle)};
      newInstructionQueue.push(turnInstruction);
    }
  }
  //build forward instruction
  if (L != 0) {
    Instruction forwardInstruction = {"FW", String(L)};
    newInstructionQueue.push(forwardInstruction);
  }
  //add the rest of instruction queue
  while (!instructionQueue.empty()) {
    newInstructionQueue.push(instructionQueue.front());
    instructionQueue.pop();
  }
  //replace old instruction queue with new one
  instructionQueue = newInstructionQueue;
}

//Decodes an instruction into a String that can be sent through UART
String decodeInstr(const Instruction &input) {
  String decoded = "";
  if (input.command == "FW") {
    decoded += "f";
  } else if (input.command == "BW") {
    decoded += "b";
  } else if (input.command == "CL") {
    decoded += "r";
  } else if (input.command == "CC") {
    decoded += "l";
  } else if (input.command == "SP") {
    decoded += "s";
  } else if (input.command == "ST") {
    decoded += "x";
  } else if (input.command == "CO") {
    //Convert the received coordinates to a Turn instruction and a Forward instruction
    //extract X and Y coordinates from input.value
    int separatorIndex;
    for (int indexC = 0; indexC < input.value.length(); indexC++) {
      if (input.value.charAt(indexC) == ':') {
        separatorIndex = indexC;
      }
    }
    long X = input.value.substring(0, separatorIndex).toInt();
    long Y = input.value.substring(separatorIndex + 1).toInt();
    //Create the equivalent FW and Turn instructions and insert at front of the instruction queue
    instructionQueue.pop();
    insertFrontInstructionQueue(X, Y);
    //Return the newly inserted instruction
    if (!instructionQueue.empty()) {
      return decodeInstr(instructionQueue.front());
    } else {
      return "";
    }
  } else {
    return "";
  }
  decoded += input.value;
  return decoded;
}


#endif
