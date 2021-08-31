/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "string.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern char COOR[25];

float X = .0, Y = .0, Z = .0;
extern float ground_lat;
extern float ground_lng;

extern uint8_t gps;

uint8_t cont = 0;

uint8_t fall = 0;
uint8_t cal_dis = 0;
uint8_t GS_LC = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char RX2_BUF[80] = {0};
char NEXT_BUFF[100] = { 0 };
uint8_t RX2 = 0;

extern char RX1_BUF[100];

char LAT[9];
char LONG[10];
char COOR[25];
char TIME[10];

int time = 0;

extern uint8_t c1;
extern uint8_t gps_size;

uint8_t ADDH = 0x6;
uint8_t ADDL = 0x4A;
uint8_t CHN = 0xA;
uint8_t MODE = 1;

void LORA_READ_PARAMETER(void);
void LORA_CONFG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE);
float calcoor(float x);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  LORA_READ_PARAMETER();
  LORA_CONFG( ADDH, ADDL, CHN, MODE); // HIGH ADDRESS, LOW ADDRESS, CHANNEL, MODE (0 : TRANSPARENT, 1 : FIXED)
  for(uint8_t i = 0 ; i < 8 ; i++){
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	  HAL_Delay(50);
  }
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  HAL_TIM_Base_Start_IT(&htim2);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
 __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (gps == 1) {
			for (uint8_t i = 0; i < gps_size; i++) {
				if (RX1_BUF[i] == '$' && RX1_BUF[i + 1] == 'G'
						&& RX1_BUF[i + 2] == 'P' && RX1_BUF[i + 3] == 'G'
						&& RX1_BUF[i + 4] == 'G' && RX1_BUF[i + 5] == 'A'
						&& RX1_BUF[i + 6] == ',') {
					uint8_t v = 0, pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0,
							pos5 = 0, ok1 = 0, ok2 = 0, ok3 = 0, ok4 = 0;
					for (uint8_t n = 0; n < sizeof(RX1_BUF); n++) {  //  ONEMLI
						if (RX1_BUF[n] == ',' && n > i + 5)
							v++;
					}
					if (v >= 5) { // new condition
						v = 0;
						for (uint8_t n = 0; n < sizeof(RX1_BUF); n++) { //  ONEMLI
							if (RX1_BUF[n] == ',' && n > i + 5)
								v++;
							if (v == 1 && ok1 == 0) {
								pos1 = n;
								ok1 = 1;
							}
							if (v == 2 && ok2 == 0) {
								pos2 = n;
								for (uint8_t count = 0; count < pos2 - pos1;
										count++)
									TIME[count] = RX1_BUF[pos1 + count + 1];
								ok2 = 1;
							}
							if (v == 3 && ok3 == 0) {
								pos3 = n;
								for (uint8_t count = 0; count < pos3 - pos2;
										count++)
									LAT[count] = RX1_BUF[pos2 + count + 1];
								ok3 = 1;
							}
							if (v == 4 && ok4 == 0) {
								pos4 = n;
								ok4 = 1;
							}
							if (v == 5) {
								pos5 = n;
								for (uint8_t count2 = 0; count2 < pos5 - pos4;
										count2++)
									LONG[count2] = RX1_BUF[pos4 + count2 + 1];

								strcpy(TIME, TIME);
								strcpy(LAT, LAT);
								strcpy(LONG, LONG);
								time = atof(TIME);
								ground_lat = calcoor(atof(LAT));
								ground_lng = calcoor(atof(LONG));
								if (/*ground_lat >= 10. && ground_lng >= 10.*/ 1) {
									sprintf(COOR, ":%2.7f:%2.7f\n", ground_lat,
											ground_lng);

//									sprintf(NEXT_BUFF, "A:%s%s", NEXT_BUFF, COOR);
//									HAL_UART_Transmit(&huart3, (uint8_t*) NEXT_BUFF,
//									 sizeof(NEXT_BUFF), 1000);
									cal_dis = 1;
								}
								for (uint8_t c = 0; c < sizeof(RX1_BUF); c++)
									RX1_BUF[c] = '\0';
								break;
							}
						}
						break;
					}
				}
			}
			gps_size = 0;
			gps = 0;
		}
		if (RX2 == 1) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);

			for(uint8_t i = 0 ; i < sizeof(NEXT_BUFF) ; i++)
				NEXT_BUFF[i] = '\0';

			RX2 = 0;
			uint8_t m = 0;
			uint8_t go = 0;
			for (uint8_t k = 0; k < sizeof(RX2_BUF); k++) {
				if (RX2_BUF[k] == ':' && go == 0) {
					go = 1;
				}
				if (go == 1) {
					NEXT_BUFF[m] = RX2_BUF[k - 1];
					m++;
				}
				if (go == 1 && RX2_BUF[k] == '\0')
					break;
			}

			for(uint8_t i = 0 ; i < sizeof(RX2_BUF) ; i++)
				RX2_BUF[i] = '\0';



			for (uint8_t j = 0; j < sizeof(NEXT_BUFF) + 2; j++) {
				uint8_t h = sizeof(NEXT_BUFF) + 2 - j;
				NEXT_BUFF[h] = NEXT_BUFF[h - 2];
			}
			NEXT_BUFF[0] = 'A';
			NEXT_BUFF[1] = ':';
			uint8_t dev = 0;
			for (uint8_t i = 0; i < sizeof(NEXT_BUFF); i++) {
				if (NEXT_BUFF[i] == '\n') {
					NEXT_BUFF[i] = ':';
					NEXT_BUFF[i + 1] = 'B';
					//NEXT_BUFF[i + 2] = '\n';
					dev = 1;
				}
				if (dev == 1) {
					NEXT_BUFF[i + 2] = '\0';
				}
			}
	  			HAL_UART_Transmit(&huart3, (uint8_t*) NEXT_BUFF, sizeof(NEXT_BUFF),
	  					1000);

	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);

		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 69;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M0_Pin|M1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M0_Pin M1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void LORA_CONFG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, SET);
	HAL_Delay(50);

	char cfg_buff[6] = {0};
	enum lora{Transparent, Fixed} mode;
	mode = MODE;

	cfg_buff[0] = 0xC0;  // header for saving paramater when power down C0
	cfg_buff[1] = ADDH;  // high address
	cfg_buff[2] = ADDL;  // low address
	cfg_buff[3] = 0x19;  // SPED (parity, baud, data rate)
	cfg_buff[4] = CHN;   // channel

	switch(mode){
	case Transparent:
		cfg_buff[5] = 0x44;  // option
		break;
	case Fixed:
		cfg_buff[5] = 0xC4;  // option
		break;
	default:
		cfg_buff[5] = 0x44;  // option
	}

	HAL_UART_Transmit(&huart2, (uint8_t*) cfg_buff, 6, 1000);

	HAL_Delay(25);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, RESET);
	HAL_Delay(25);
}

void LORA_READ_PARAMETER()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, SET);
	HAL_Delay(50);

	char buff_read[6] = {0};
	buff_read[0] = 0xC1;
	buff_read[1] = 0xC1;
	buff_read[2] = 0xC1;

	HAL_UART_Transmit(&huart2, (uint8_t*) buff_read, 3, 1000);
	HAL_UART_Receive(&huart2, (uint8_t*) buff_read, 6, 1000);

	HAL_Delay(25);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, RESET);
	HAL_Delay(25);
}

float calcoor(float x)
{
	float a = (int)x / 100;
	float b = (x - (a * 100.0)) / 60.0;
	return a+b;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
