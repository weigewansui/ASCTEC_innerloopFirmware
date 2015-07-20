/**
 * 
 * Files for functions that generate direct motor control command from 
 * desired Euler angle and Euler rate input
 * 
 */
#include "Gen_motor_cmmd.h"
#include "LL_HL_comm.h"

//Motor speed at the nominal hovering point
static short motor_speed_nominal = 72;
short motor_speed_deviation[4];
short current_roll, current_pitch, current_yaw;

unsigned char* GetMotorCmmdFromUData (struct WO_DESIRED_INPUT desired_data) {

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


/**
 * Use value from the serial interface to generate motor command
 *
 * Use WO_DESIRED_Input structure value to generate LL_1khz_control_input.direct_motor_control
 * 
 * Then WO_DESIRED_Input structure contains:
 * 
 * 	unsigned char pitch_angle;
	unsigned char roll_angle;
	unsigned char yaw_angle;

	unsigned char pitch_vel;
	unsigned char roll_vel;
	unsigned char yaw_vel;
 *
 * 
 */

#define KP_ROLL 1
#define KD_ROLL 1

#define KP_PITCH 1
#define KD_PITCH 1

#define KP_YAW 1
#define KD_YAW 1

// calculate via m/(8*Kf*Wh)
#define K_THRUST 1

/**
 * calculate desired motor speed command directly from global variables
 * 
 */

void GenMotorCmmd_Linear (void) {

// select feedback value from vicon and IMU integration 
	if(WO_SDK.vicon_available) {

		current_roll = WO_Feedback.vicon_roll;
		current_pitch = WO_Feedback.vicon_pitch;
		current_yaw = WO_Feedback.vicon_yaw;

	} else {

		current_roll = RO_ALL_Data.angle_roll;
		current_pitch = RO_ALL_Data.angle_pitch;
		current_yaw = RO_ALL_Data.angle_yaw;

	}

	//generate deviation from PD controller
	motor_speed_deviation[0] = K_THRUST * WO_DESIRED_Input.accel_z;
	motor_speed_deviation[1] = KP_ROLL * (WO_DESIRED_Input.roll_angle - current_roll) +
								KD_ROLL * (WO_DESIRED_Input.roll_vel - RO_ALL_Data.angvel_roll);

	motor_speed_deviation[2] = KP_PITCH * (WO_DESIRED_Input.pitch_angle - current_pitch) + 
								KD_PITCH * (WO_DESIRED_Input.pitch_vel - RO_ALL_Data.angvel_pitch);
	motor_speed_deviation[3] = KP_YAW * (WO_DESIRED_Input.yaw_angle - current_yaw) + 
								KD_YAW * (WO_DESIRED_Input.yaw_vel - RO_ALL_Data.angvel_yaw); 

	CalMotorSpeedFromDev(&motor_speed_deviation);

}

/**
 * get individual motor command from deviations of nominal points
 * @param SpeedDeviation a array of deviations
 *        SpeedDeviation[0] Omega_F
 *        SpeedDeviation[1] Omega_phi
 *        SpeedDeviation[2] Omega_theta
 *        SpeedDeviation[3] Omega_psi
 *        
 */
void CalMotorSpeedFromDev (unsigned char** SpeedDeviation) {

	LL_1khz_control_input.direct_motor_control[0] = motor_speed_nominal + (*SpeedDeviation)[0] - (*SpeedDeviation)[2] + (*SpeedDeviation)[3];
	LL_1khz_control_input.direct_motor_control[1] = motor_speed_nominal + (*SpeedDeviation)[0] + (*SpeedDeviation)[1] - (*SpeedDeviation)[3];
	LL_1khz_control_input.direct_motor_control[2] = motor_speed_nominal + (*SpeedDeviation)[0] + (*SpeedDeviation)[2] + (*SpeedDeviation)[3];
	LL_1khz_control_input.direct_motor_control[3] = motor_speed_nominal + (*SpeedDeviation)[0] - (*SpeedDeviation)[1] - (*SpeedDeviation)[3];

}
