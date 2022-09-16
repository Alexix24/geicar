#ifndef __can_H
#define __can_H

#define ID_US 0x211
#define ID_SSB_DATAS 0x200    //Battery level + Motors feedback
#define ID_GPS 0x223
#define ID_IMU 0x232
#define ID_CALIBRATION_MODE 0x300

#define ID_MOTORS_CMD 0x100


#define CALIBRATION_REQUEST	0x1		//frame[0]
#define CALIBRATION_IN_PROGRESS 0x2	//frame[0]
#define CALIBRATION_SUCCESS 0x3		//frame[0]
#define CALIBRATION_FAIL 0x4		//frame[0]

#define CALIBRATION_USER_NEED 0x1	//frame[1]


#endif /*__ can_H */