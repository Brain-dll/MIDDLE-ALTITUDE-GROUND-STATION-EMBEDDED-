/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdlib.h>
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#define PI 3.14159265358979323846

char buff1 = '\0', buff2 = '\0';
char RX1_BUF[100];
extern char RX2_BUF[80];
extern uint8_t RX2;

extern uint8_t stop;
extern uint8_t newrecord;
extern uint8_t pause;

extern char NEXT_BUFF[100];  //***

char LAT[9];
char LONG[10];
char COOR[25];
char TIME[10];

float ground_lat = .0;
float ground_lng = .0;
int time = 0;

uint8_t c1 = 0, n = 0;
extern uint8_t mgn;

char RX3_buff[4] = {0};
char buff = '\0';
uint8_t count1 = 0;
uint8_t run = 0;
extern uint8_t cal_dis;

extern uint8_t recal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
float calcoor(float x);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */


  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */

    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */

    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */

    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */
  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  mgn = 1;
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &buff1, 1);
	RX1_BUF[c1] = buff1;
	if (RX1_BUF[c1] == '\n') {
		for (uint8_t i = 0; i < c1; i++) {
			if (RX1_BUF[i] == '$' && RX1_BUF[i + 1] == 'G'
					&& RX1_BUF[i + 2] == 'P' && RX1_BUF[i + 3] == 'G'
					&& RX1_BUF[i + 4] == 'G' && RX1_BUF[i + 5] == 'A'
					&& RX1_BUF[i + 6] == ',') {
				uint8_t v = 0, pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0, pos5 = 0,
						ok1 = 0, ok2 = 0, ok3 = 0, ok4 = 0;
				for (uint8_t n = 0; n < sizeof(RX1_BUF); n++) {
					if (RX1_BUF[n] == ',' && n > i + 5)
						v++;
					if (v == 1 && ok1 == 0) {
						pos1 = n;
						ok1 = 1;
					}
					if (v == 2 && ok2 == 0) {
						pos2 = n;
						for (uint8_t count = 0; count < pos2 - pos1; count++)
							TIME[count] = RX1_BUF[pos1 + count + 1];
						ok2 = 1;
					}
					if (v == 3 && ok3 == 0) {
						pos3 = n;
						for (uint8_t count = 0; count < pos3 - pos2; count++)
							LAT[count] = RX1_BUF[pos2 + count + 1];
						ok3 = 1;
					}
					if (v == 4 && ok4 == 0) {
						pos4 = n;
						ok4 = 1;
					}
					if (v == 5) {
						pos5 = n;
						for (uint8_t count2 = 0; count2 < pos5 - pos4; count2++)
							LONG[count2] = RX1_BUF[pos4 + count2 + 1];

						strcpy(TIME, TIME);
						strcpy(LAT, LAT);
						strcpy(LONG, LONG);
						time = atof(TIME);
						ground_lat = calcoor(atof(LAT));
						ground_lng = calcoor(atof(LONG));
						sprintf(COOR, ":%2.7f:%2.7f\n", ground_lat, ground_lng);
						/*HAL_UART_Transmit(&huart3, (uint8_t*) COOR,
						 sizeof(COOR), 1000);*/
						uint32_t lat0next = 0;
						uint32_t lng0next = 0;
						lat0next = (uint32_t) (ground_lat * 10000000.0);
						lng0next = (uint32_t) (ground_lng * 10000000.0);
//						NXT_SEND_VAL("page1.x8.val", lat0next);
//						NXT_SEND_VAL("page1.x9.val", lng0next);
						cal_dis = 1;
						for (uint8_t c = 0; c < sizeof(RX1_BUF); c++)
							RX1_BUF[c] = '\0';
						break;
					}
				}
				break;
			}
		}
		c1 = 0;
	}
	c1++;
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &buff2, 1);
	if (buff2 != '\n' && buff2 != '\0' ) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		RX2_BUF[n] = buff2;
		n++;
	} else if(buff2 == '\n') {
		uint8_t A = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		for (n; n < sizeof(RX2_BUF); n++) {
			RX2_BUF[n] = COOR[A];
			if (COOR[A] == '\0') {
				break;
			}
			A++;
		}
		n = 0;
		//HAL_UART_Transmit(&huart3, (uint8_t*) RX2_BUF, sizeof(RX2_BUF), 1000);  // For hard fault interrupt, you have to assign priority UART ports
		RX2 = 1;
  	}
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  HAL_UART_Receive_IT(&huart3, (uint8_t*) &buff, 1);

  	if (buff == '$')
  		run = 1;
  	else if(buff == 0x00){
  		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
  	}
  	else if(buff == 0x1A){
  	  		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  	  	}
  	if (run == 1) {
  		RX3_buff[count1] = buff;
  		count1++;
  		if (buff == '&') {
  			count1 = 0;
  			if (RX3_buff[1] == '0' && RX3_buff[2] == 0x02)
  				recal = 1;
  			else if (RX3_buff[1] == '0' && RX3_buff[2] == 0x05)
  				newrecord = 1;
  			else if (RX3_buff[1] == '0' && RX3_buff[2] == 0x03){
  				if(pause == 0)
  					pause = 1;
  				else
  					pause = 0;
  			}
  			else if (RX3_buff[1] == '0' && RX3_buff[2] == 0x04)
  				stop = 1;
  			for (uint8_t i = 0; i < sizeof(RX3_buff); i++)
  				RX3_buff[i] = '\0';
  		}
  		if (buff == '&')
  			run = 0;
  	}
  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
float calcoor(float x)
{
	float a = (int)x / 100;
	float b = (x - (a * 100.0)) / 60.0;
	return a+b;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
