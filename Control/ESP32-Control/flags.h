#ifndef flags_h
#define flags_h

//Drive flags
bool driveWaiting;                          //true if Drive is ready to receive an instruction

//Vision flags
bool visionOverride;                        //true when we need to avoid an obstacle
bool playingRoutine;                        //true is playing the obstacle avoidance routine

#endif
