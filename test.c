#include <stdio.h>
#include <stdlib.h>
#include "Gen_motor_cmmd.c"


int main() {

	struct WO_DESIRED_INPUT wo_x;

	wo_x.pitch_angle = 1;
	wo_x.roll_angle = 1;
	wo_x.yaw_angle = 0;

	unsigned char* a = getMotorCmmdFromUData(wo_x);

	printf("%d\n", __LINE__);
	printf("%d\n", __LINE__);

	printf("%d, %d, %d, %d\n", a[0], a[1], a[2], a[3]);


}


unsigned char* Test(unsigned char* test2) {

	unsigned char* ret = (unsigned char*) malloc(4*sizeof(unsigned char));
	printf("%d\n", __LINE__);


	for(int i = 0; i < 4; i++) {
	printf("%d\n", __LINE__);

		ret[i] = test2[i];
	}

	return ret;

}

