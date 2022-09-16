#include "../include/car_control/propulsionCmd.h"
#include <stdint.h>

#define STOP 50

int * propulsionCmd (uint8_t requestedCarVelocity, uint8_t currentCarVelocity, uint8_t& leftRearSpeedCmd, uint8_t& rightRearSpeedCmd){

    uint8_t speedError = requestedCarVelocity - currentCarVelocity;

    // --- Write the command here ---
    leftRearSpeedCmd = requestedCarVelocity;
    rightRearSpeedCmd = requestedCarVelocity;
    //---------

    
    return 0;
}