/**
 * 
 * Files for functions that generate direct motor control command from 
 * desired Euler angle and Euler rate input
 * 
 */
#include "Gen_motor_cmmd.h"
#include "LL_HL_comm.h"

//Motor speed at the nominal hovering point
static short motor_speed_nominal = 72*64; //multiply by 64 for real RPM
short motor_speed_deviation[4];
float current_roll, current_pitch, current_yaw;
float current_roll_vel, current_pitch_vel, current_yaw_vel;
float desired_roll, desired_pitch, desired_yaw;

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

/**
 * KP_ROLL = I_xx/(4*Kf*L*Wh)*KP_LQR/coefficient
 * KP_LQR = 2.4142135
 * KD_LQR = 2.210
 *
 * I_xx = 0.00367556974
 * I_yy = 0.00367556974
 * I_zz = 0.00703095144 
 *
 * Kf = 6.11e-8
 * Km = 1.5e-9
 * Wh = 72*64
 * L = 170 mm = 0.17m
 *
 * KP_YAW = KP_LQR*I_zz/(8*K_m*Wh)/coefficient
 * 
 */
#define KP_ROLL 0.7242
#define KD_ROLL 0.66294

#define KP_PITCH 0.7242
#define KD_PITCH 0.66294

#define KP_YAW 4.7964
#define KD_YAW 4.39069


// calculate via m/(8*Kf*Wh)/(coefficient)
// m = 0.5200
// Kf = 6.11e-8
// Wh = 72*64
// Coefficient = 64 divide by to get RPM command
// The z axis is poiting downwards, so it's minus
#define K_THRUST -3.6

/**
 * calculate desired motor speed command directly from global variables
 * 
 */

void GenMotorCmmd_Linear (void) {

// select feedback value from vicon and IMU integration 
	if(WO_SDK.vicon_available) {

		current_roll = WO_Feedback.vicon_roll/20000.000;
		current_pitch = WO_Feedback.vicon_pitch/20000.000;
		current_yaw = WO_Feedback.vicon_yaw/20000.000;

	} else {

		current_roll = 0.01745329*RO_ALL_Data.angle_roll/1000.000;
		current_pitch = 0.01745329*RO_ALL_Data.angle_pitch/1000.000;
		current_yaw = 0.01745329*RO_ALL_Data.angle_yaw/1000.000;

	}

	current_roll_vel = 0.00026878067*RO_ALL_Data.angvel_roll;
	current_pitch_vel = 0.00026878067*RO_ALL_Data.angvel_pitch;
	current_yaw_vel = 0.00026878067*RO_ALL_Data.angvel_yaw;
	/**
	 * WO_DESIRED_Input angles unit: rad*20000
	 * WO_DESIRED_Input vel unit: rad*10000
	 *
	 * current_(angle) unit: degree*1000
	 * 
	 * RO_ALL_Data.angvel_(angle) unit: 0.0154 degree/s
	 *
	 *
	 */
	
	//generate deviation from PD controller
	motor_speed_deviation[0] = K_THRUST * WO_DESIRED_Input.accel_z;
	motor_speed_deviation[1] = KP_ROLL * (WO_DESIRED_Input.roll_angle - current_roll) +
								KD_ROLL * (WO_DESIRED_Input.roll_vel - current_roll_vel);

	motor_speed_deviation[2] = KP_PITCH * (WO_DESIRED_Input.pitch_angle - current_pitch) + 
								KD_PITCH * (WO_DESIRED_Input.pitch_vel - current_pitch_vel);
	motor_speed_deviation[3] = KP_YAW * (WO_DESIRED_Input.yaw_angle - current_yaw) + 
								KD_YAW * (WO_DESIRED_Input.yaw_vel - current_yaw_vel); 

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
 * The linear combination matrix is:
 *
 * 		1	0	1	-1
 *   	1	0	-1	-1
 *    	1	1	0	1
 *     	1	-1	0	1
 * 
 */
void CalMotorSpeedFromDev (unsigned char** SpeedDeviation) {

	LL_1khz_control_input.direct_motor_control[0] = motor_speed_nominal + (*SpeedDeviation)[0] + (*SpeedDeviation)[2] - (*SpeedDeviation)[3];
	LL_1khz_control_input.direct_motor_control[1] = motor_speed_nominal + (*SpeedDeviation)[0] - (*SpeedDeviation)[2] - (*SpeedDeviation)[3];
	LL_1khz_control_input.direct_motor_control[2] = motor_speed_nominal + (*SpeedDeviation)[0] + (*SpeedDeviation)[1] + (*SpeedDeviation)[3];
	LL_1khz_control_input.direct_motor_control[3] = motor_speed_nominal + (*SpeedDeviation)[0] - (*SpeedDeviation)[1] + (*SpeedDeviation)[3];

}
