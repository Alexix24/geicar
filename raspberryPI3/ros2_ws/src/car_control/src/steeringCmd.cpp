#include "../include/car_control/steeringCmd.h"

#include <stdint.h>
#include "math.h"


#define STOP 50
#define MAX_SPEED_LEFT 0
#define MAX_SPEED_RIGHT 100

#define TOLERANCE_ANGLE 3



//return the speed command to reach the angle passed in argument
int steeringCmd(uint8_t requestedAngle, uint8_t currentAngle, uint8_t & steeringSpeedCmd){

	int errorAngle = currentAngle - requestedAngle;

    //Command's calculation
	if (abs(errorAngle)<TOLERANCE_ANGLE){
		steeringSpeedCmd = STOP;
	}
	else {
		if (errorAngle>0) {
			steeringSpeedCmd = MAX_SPEED_LEFT;
		}
		else {
			steeringSpeedCmd = MAX_SPEED_RIGHT;
		}
	}

    return errorAngle;
}