/*
 * nextion.c
 *
 *  Created on: 1 Ağu 2021
 *      Author: Beytu
 */

#include "nextion.h"

uint8_t cmd_nxt[3] = {0xFF, 0xFF, 0xFF};
int len = 0;

void NXT_SEND_VAL(char* ID, uint32_t val)
{
	char buff[50];
	len = sprintf(buff, "%s=%d", ID, val);
	HAL_UART_Transmit(&huart3, (uint8_t*)buff, len, 1000);
	HAL_UART_Transmit(&huart3, cmd_nxt, 3, 100);
}

void NXT_SEND_SVAL(char* ID, int val)
{
	char buff[50];
	len = sprintf(buff, "%s=%d", ID, val);
	HAL_UART_Transmit(&huart3, (uint8_t*)buff, len, 1000);
	HAL_UART_Transmit(&huart3, cmd_nxt, 3, 100);
}

void NXT_SEND_VALFLOAT(char* ID, float val)
{
	char buff[50];
	len = sprintf(buff, "%s=\"%.2f\"", ID, val);
	HAL_UART_Transmit(&huart3, (uint8_t*)buff, len, 1000);
	HAL_UART_Transmit(&huart3, cmd_nxt, 3, 100);
}

void NXT_SEND_STR(char* ID)
{
	char buff[50];
	int len = sprintf(buff, "%s", ID);
	HAL_UART_Transmit(&huart3, (uint8_t*) buff, len, 1000);
	HAL_UART_Transmit(&huart3, cmd_nxt, 3, 100);
}
void NXT_SEND_TXT(char* ID, uint16_t val)
{
	char buff[50];
	len = sprintf(buff, "%s=\"%d\"", ID, val);
	HAL_UART_Transmit(&huart3, (uint8_t*)buff, len, 1000);
	HAL_UART_Transmit(&huart3, cmd_nxt, 3, 100);
}

void NXT_SEND_TXTBX(char* ID, char* txt)
{
	char buff[90];
	len = sprintf(buff, "%s=\"%s\"", ID, txt);
	HAL_UART_Transmit(&huart3, (uint8_t*)buff, len, 1000);
	HAL_UART_Transmit(&huart3, cmd_nxt, 3, 100);
}

void NXT_Disable_Error(char* txt)
{
	char buff[8];
	len = sprintf(buff, "%s", txt);
	HAL_UART_Transmit(&huart3, cmd_nxt, 3, 100);
	HAL_UART_Transmit(&huart3, (uint8_t*)buff, len, 1000);
	HAL_UART_Transmit(&huart3, cmd_nxt, 3, 100);
}



