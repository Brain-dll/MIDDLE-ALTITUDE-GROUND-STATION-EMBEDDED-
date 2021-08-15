/*
 * MAGNETO.h
 *
 *  Created on: 23 Tem 2021
 *      Author: Beytu
 */

#ifndef INC_MAGNETO_H_
#define INC_MAGNETO_H_

#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

#define MGN_density 6.
#define MGN_default_Angle 0//90  // for display compass

/*
#define MGN_WRITE_ADD 0x1A
#define MGN_READ_ADD 0x1B
*/
#define MGN_WRITE_ADD 0x3C
#define MGN_READ_ADD 0x3D



void magneto_Init(uint8_t confg_c);
void Read_mgn(void);
float Read_compass(void);
//void NXT_SEND_PROG(char* ID, uint16_t val);

#endif /* INC_MAGNETO_H_ */
