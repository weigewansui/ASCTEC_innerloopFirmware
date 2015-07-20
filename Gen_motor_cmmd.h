
#ifndef GEN_MOTOR_CMMD_
#define GEN_MOTOR_CMMD_

#include "sdk.h"

unsigned char* GetMotorCmmdFromUData (struct WO_DESIRED_INPUT);
void GenMotorCmmd_Linear (void);
void CalMotorSpeedFromDev (unsigned char**);

#endif
