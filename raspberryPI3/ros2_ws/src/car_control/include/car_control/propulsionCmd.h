#include <cstdint>

#ifndef __propulsionCmd_H
#define __propulsionCmd_H


int * propulsionCmd (uint8_t requestedCarVelocity, uint8_t currentCarVelocity, uint8_t & leftRearSpeedCmd, uint8_t & rightRearSpeedCmd);



#endif /*__ propulsionCmd_H */