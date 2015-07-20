/**
 * 
 * Files for functions that generate direct motor control command from 
 * desired Euler angle and Euler rate input
 * 
 */

#include "Gen_motor_cmmd.h"

unsigned char* getMotorCmmdFromUData (struct WO_DESIRED_INPUT desired_data) {

	// testing
	unsigned char* DMC_cmmd = (unsigned char*)malloc(4*sizeof(unsigned char));

	int i;

	for (i = 0; i < 4; i++) {
		
		DMC_cmmd[i] = 0;

	}

	if (desired_data.roll_angle == 1) {

		DMC_cmmd[0] = 20;
	} else DMC_cmmd[0] = 0;

	if (desired_data.pitch_angle == 1) DMC_cmmd[1] = 20; else DMC_cmmd[1] = 0;
	if (desired_data.yaw_angle == 1) DMC_cmmd[2] = 20;  else DMC_cmmd[2] = 0;
	if (desired_data.roll_vel == 1) DMC_cmmd[3] = 20; else DMC_cmmd[3] = 0;

	return DMC_cmmd;

}
