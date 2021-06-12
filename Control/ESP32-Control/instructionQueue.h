#ifndef instructionqueue_h
#define instructionqueue_h

#include "instruction.h"
#include <queue>


// IMPORTANT: All distances are in millimeters.


// -------- INSTRUCTION QUEUE CONSTANTS --------

//Obstacle detection trigger threshold
const long obstacleDetectionThreshold = 100;



// -------- INSTRUCTION QUEUE VARIABLES --------


//Normal Instruction Queue
std::queue<Instruction> instructionQueue;
Instruction lastInstruction;

//Stop Instruction
const Instruction stopInstr = {"ST", ""};

//Obstacle avoidance path parameters
const int sizeOfBall = 50;
const int avoidanceCost = (obstacleDetectionThreshold * 2) + sizeOfBall;
const String pathLength = String(avoidanceCost);
const String pathWidth = String(avoidanceCost / 2);

//Obstacle Avoidance Routine Queue
const std::deque<Instruction> initQueue{{"CL", "90"}, {"FW", pathWidth}, {"CC", "90"}, {"FW", pathLength}, {"CC", "90"}, {"FW", pathWidth}, {"CL", "90"}};
const std::queue<Instruction> avoidRoutine(initQueue);
std::queue<Instruction> currentAvoidRoutine;
Instruction lastRoutineInstruction;



// -------- INSTRUCTION QUEUE FUNCTIONS --------

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
  } else {
    return "";
  }
  decoded += input.value;
  return decoded;
}


#endif
