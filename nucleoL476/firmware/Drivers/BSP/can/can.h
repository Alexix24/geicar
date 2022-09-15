/**
 ******************************************************************************
 * File Name          : CAN.h
 * Description        : This file provides code for the configuration
 *                      of the CAN instances.
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CAN_H_
#define _CAN_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

typedef void (*CanRxCallback)(CAN_RxHeaderTypeDef *rxHeader, uint8_t data[]);

#define CAN_ID_GPS1 	0x272 	// GPS Lat DD
#define CAN_ID_GPS2 	0x282 	// GPS Long DD
#define CAN_ID_GPS3 	0x292 	// GPS Long DD
#define CAN_ID_GPS4 	0x302 	// Quality (no fix, rtk fixed, rtk float ...), hAcc , vAcc
#define CAN_ID_IMU1  0x232	// IMU data : Magnetic field
#define CAN_ID_IMU2  0x242  // IMU data : Angular velocity
#define CAN_ID_IMU3  0x252  // IMU data : Linear acceleration
#define CAN_ID_IMU4  0x263  // IMU data : General data (temperature, pressure, humidity)

#define CAN_ID_COMM_CHECKING 0x410

#define COMM_CHECKING_REQUEST 0x1 //frame[0]
#define COMM_CHECKING_ACK 0x2	  //frame[1]
/**
 * @brief  Configures the CAN.
 * @param  None
 * @retval None
 */
void CAN_Init(void);

/*void CAN_Send(uint8_t std_id, uint8_t ext_id, uint8_t len, uint8_t data[]);*/

void CAN_Send(uint8_t* data, uint32_t id);

void CAN_AddRXCallback(CanRxCallback callback);


#ifdef __cplusplus
}
#endif
#endif /* _CAN_H_ */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
