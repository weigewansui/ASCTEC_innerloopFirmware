
#ifndef GEN_MOTOR_CMMD
#define GEN_MOTOR_CMMD

#include "sdk.h"

unsigned char* GetMotorCmmdFromUData (struct WO_DESIRED_INPUT);
void GenMotorCmmd(void);
void CalMotorSpeedFromDev (unsigned char**);

#endif
