/*
 * nextion.h
 *
 *  Created on: 1 Ağu 2021
 *      Author: Beytu
 */

#ifndef INC_NEXTION_H_
#define INC_NEXTION_H_

#include "stm32f1xx_hal.h"
#include "stdio.h"

extern UART_HandleTypeDef huart3;

void NXT_SEND_VAL(char* ID, uint32_t val);
void NXT_SEND_STR(char* ID);
void NXT_SEND_TXT(char* ID, uint16_t val);
void NXT_SEND_TXTBX(char* ID, char* txt);
void NXT_SEND_VALFLOAT(char* ID, float val);
void NXT_SEND_SVAL(char* ID, int val);
void NXT_Disable_Error(char* txt);

#endif /* INC_NEXTION_H_ */
