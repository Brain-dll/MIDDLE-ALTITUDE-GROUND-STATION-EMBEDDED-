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
#include "MAGNETO.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include <stdlib.h>
#include "MPU-6050.h"
#include "nextion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void Data_Parsing(void);
void Data_Convert(void);
void Data_ToNextion(void);
double deg2rad(float deg);
float Cal_distGPS(float lat, float lng, float ground_latt, float ground_lngg);
void finding_maxmin(void);
double rad2deg(float rad);
float GetAngle(float lat, float lng, float ground_latt, float ground_lngg);

#define cal_num 150
float compass_val = .0;
float X = .0, Y = .0, Z = .0;
extern float ground_lat;
extern float ground_lng;
float EPH_lat = .0;
float EPH_lng = .0;
float PL_lat = .0;
float PL_lng = .0;

float R = 6378.1; // Radius of the earth in km
float dLat = .0; // deg2rad below
float dLon = .0;
float a = .0;
float c = .0;
float d = .0; // Distance in km
float distance = .0;
float EPH_distance = .0;
float PL_distance = .0;
float EPH_Angle = .0;
float PL_Angle = .0;

uint8_t mgn = 0;
uint8_t recal = 0;
uint8_t stop = 0;
uint8_t newrecord = 0;
uint8_t pause = 0;
uint8_t cont = 0;

char ID = '\0';
char sn[2] = {0};
char lat1[10] = {0};
char lng1[10] = {0};
char alt[7] = {0};
char vlc[6] = {0};
char rollpitch[13] = {0};
char apg[1] = {0};
char mn[1] = {0};
uint8_t fall = 0;
uint8_t cal_dis = 0;
uint8_t GS_LC = 0;

extern uint32_t SystemCoreClock;
uint32_t Systemlock = 0;

float vlcf = .0, altf = .0;
int alti = 0, vlci = 0, EPHmax_alti = 0, EPHmax_vlci = 0 , EPHmin_vlci = 0, PLmax_alti = 0, PLmax_vlci = 0 , PLmin_vlci = 0;
int EPHalti = 0, EPHvlci = 0, PLalti = 0, PLvlci = 0, PLmin_alti = 1000, EPHmin_alti = 1000;
// PLmin_alti  EPHmin_alti

int16_t speedometer = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char RX2_BUF[120] = {0};
char NEXT_BUFF[120] = { 0 };
uint8_t RX2 = 0;

uint8_t ADDH = 0x6;
uint8_t ADDL = 0x4A;
uint8_t CHN = 0xA;
uint8_t MODE = 1;

void LORA_READ_PARAMETER(void);
void LORA_CONFG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //newrecord = 1;
  //NXT_SEND_SVAL("page0.t19.pco", 2016);

  LORA_READ_PARAMETER();
  LORA_CONFG( ADDH, ADDL, CHN, MODE); // HIGH ADDRESS, LOW ADDRESS, CHANNEL, MODE (0 : TRANSPARENT, 1 : FIXED)
  MPU6050_Init();
  for(uint8_t i = 0 ; i < 8 ; i++){
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	  HAL_Delay(50);
  }
  //magneto_Init(cal_num);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  HAL_TIM_Base_Start_IT(&htim2);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
 __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
 NXT_Disable_Error("bkcmd=2");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Systemlock = SystemCoreClock;
		if (mgn == 1) {
			compass_val = Read_compass();
			//NXT_SEND("t0", compass_val);
			NXT_SEND_VAL("page2.z0.val", (uint32_t) compass_val);
			compass_val -= 90;
			if (compass_val < 0)
				compass_val += 360;
			NXT_SEND_TXT("page2.t7.txt", (uint16_t) compass_val);
			mgn = 0;
		}
		if (recal == 1) {
			magneto_Init(cal_num);
			recal = 0;
		}
		if (RX2 == 1) {

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
				if (go == 1 && RX2_BUF[k] == '\n')
					break;
			}
			//if(NEXT_BUFF[0] != '\0' || NEXT_BUFF[1] || '\0' || NEXT_BUFF[2] != '\0')
			NXT_SEND_TXTBX("page1.t12.txt", NEXT_BUFF);

			for (uint8_t i = 0; i < sizeof(RX2_BUF); i++)
				RX2_BUF[i] = '\0';
			Data_Parsing();
		}
		if (cal_dis == 1) {
			if (ground_lat == .0 && ground_lng == .0) {
				NXT_SEND_TXTBX("page2.t4.txt", "veri yok");
				NXT_SEND_TXTBX("page2.t6.txt", "veri yok");
			} else if (EPH_lat == .0 && EPH_lng == .0) {
				NXT_SEND_TXTBX("page2.t4.txt", "veri yok");
			} else {
				EPH_distance = Cal_distGPS(EPH_lat, EPH_lng, ground_lat,
						ground_lng);
				EPH_Angle = GetAngle(EPH_lat, EPH_lng, ground_lat, ground_lng);
				NXT_SEND_VALFLOAT("page2.t4.txt", EPH_distance);
				NXT_SEND_VALFLOAT("page2.t3.txt", EPH_Angle);
				EPH_Angle += 90.0;
				if (EPH_Angle > 360) {
					EPH_Angle -= 360;
				}
				NXT_SEND_VAL("page2.z1.val", (uint32_t) EPH_Angle);
			}
			if ((PL_lat == .0 && PL_lng == .0) || GS_LC == 1) {
				NXT_SEND_TXTBX("page2.t6.txt", "veri yok");
			} else {
				PL_distance = Cal_distGPS(PL_lat, PL_lng, ground_lat,
						ground_lng);
				PL_Angle = GetAngle(PL_lat, PL_lng, ground_lat, ground_lng);
				NXT_SEND_VALFLOAT("page2.t6.txt", PL_distance);
				NXT_SEND_VALFLOAT("page2.t5.txt", PL_Angle);
				PL_Angle += 90.0;
				if (PL_Angle > 360) {
					PL_Angle -= 360;
				}
				NXT_SEND_VAL("page2.z2.val", (uint32_t) PL_Angle);
			}

			cal_dis = 0;
		}
		if (newrecord == 1) {
			newrecord = 0;
			pause = 0;

		}
		if (pause == 0) {
		}
		if (stop == 1) {
			stop = 0;
			pause = 1;

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 35999;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M0_Pin|M1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|SDCARD_CS_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M0_Pin M1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin SDCARD_CS_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED_Pin|SDCARD_CS_Pin|BUZZER_Pin;
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

void Data_Parsing(void) {
	uint8_t first = 0, second = 0, countstr = 0;
	for (uint8_t i = 0; i < sizeof(NEXT_BUFF); i++) {
		if (NEXT_BUFF[i] == ':') {
			first = second;
			second = i;
			countstr++;
			uint8_t countstr1 = 0;
			for (first++; first < second; first++) {
				switch (countstr) {
				case 2:
					sn[countstr1] = NEXT_BUFF[first];
					countstr1++;
					break;
				case 3:
					lat1[countstr1] = NEXT_BUFF[first];
					countstr1++;
					break;
				case 4:
					lng1[countstr1] = NEXT_BUFF[first];
					countstr1++;
					break;
				case 5:
					alt[countstr1] = NEXT_BUFF[first];
					countstr1++;
					break;
				case 6:
					vlc[countstr1] = NEXT_BUFF[first];
					countstr1++;
					break;
				case 7:
					rollpitch[countstr1] = NEXT_BUFF[first];
					countstr1++;
					break;
				case 8:
					apg[countstr1] = NEXT_BUFF[first];
					countstr1++;
					break;
				case 9:
					mn[countstr1] = NEXT_BUFF[first];
					countstr1++;
					break;
				}
			}
		}
	}

	ID = NEXT_BUFF[0];

	Data_Convert();
	for (uint8_t i = 0; i < sizeof(NEXT_BUFF); i++)
		NEXT_BUFF[i] = '\0';
}

void Data_Convert(void) {
	finding_maxmin();

	if (ID == '0') {
		strcpy(lat1, lat1);
		strcpy(lng1, lng1);
		EPH_lat = atof(lat1);
		EPH_lng = atof(lng1);
	} else if (ID == '1') {
		strcpy(lat1, lat1);
		strcpy(lng1, lng1);
		PL_lat = atof(lat1);
		PL_lng = atof(lng1);
	}

	if(vlcf < 0){
		vlcf *= -1;
		fall = 1;
	}else fall = 0;

	speedometer = 336 + (int16_t)((vlcf / 300.0) * 225);
	if(speedometer > 360) speedometer -= 360;

	Data_ToNextion();
}

void Data_ToNextion(void) {

	if (EPH_lat != .0 && EPH_lng != .0) {	// EPHEMERISH son enlem, boylam verileri
		NXT_SEND_TXTBX("page3.t34.txt", lat1);
		NXT_SEND_TXTBX("page3.t35.txt", lng1);
	}
	if (PL_lat != .0 && PL_lng != .0) {		// PAYLOAD son enlem, boylam verileri
		NXT_SEND_TXTBX("page3.t36.txt", lat1);
		NXT_SEND_TXTBX("page3.t37.txt", lng1);
	}

	if (ID == '1') {
		NXT_SEND_TXTBX("page1.t14.txt", lat1);
		NXT_SEND_TXTBX("page1.t15.txt", lng1);
		NXT_SEND_TXTBX("page1.t16.txt", alt);

		NXT_SEND_SVAL("page3.x6.val", PLmax_vlci);
		NXT_SEND_SVAL("page3.x7.val", PLmin_vlci);
		NXT_SEND_SVAL("page3.x5.val", PLmax_alti);
		NXT_SEND_SVAL("page3.x9.val", PLmin_alti);

		NXT_SEND_SVAL("page1.c1.val", 1);
		NXT_SEND_SVAL("page1.c0.val", 0);
		NXT_SEND_SVAL("page1.z3.val", speedometer);
		if (fall == 1) {
			NXT_SEND_SVAL("page1.z3.pco", 31);
		} else {
			NXT_SEND_SVAL("page1.z3.pco", 63488);
		}

		NXT_SEND_SVAL("page1.x1.val", vlci);

	} else if (ID == '0') {
		NXT_SEND_TXTBX("page1.t8.txt", lat1);
		NXT_SEND_TXTBX("page1.t9.txt", lng1);
		NXT_SEND_TXTBX("page1.t13.txt", alt);

		NXT_SEND_SVAL("page3.x3.val", EPHmax_vlci);
		NXT_SEND_SVAL("page3.x4.val", EPHmin_vlci);
		NXT_SEND_SVAL("page3.x2.val", EPHmax_alti);
		NXT_SEND_SVAL("page3.x8.val", EPHmin_alti);  // PLmin_alti  EPHmin_alti

		NXT_SEND_SVAL("page1.c1.val", 0);
		NXT_SEND_SVAL("page1.c0.val", 1);
		NXT_SEND_SVAL("page1.z4.val", speedometer);
		if (fall == 1) {
			NXT_SEND_SVAL("page1.z4.pco", 31);
		} else {
			NXT_SEND_SVAL("page1.z4.pco", 63488);
		}

		NXT_SEND_SVAL("page1.x0.val", vlci);

		if (*apg == '1') {
			NXT_SEND_SVAL("page1.t10.bco", 63488);
		}
		if (*mn == '1') {
			NXT_SEND_SVAL("page1.t11.bco", 63488);
		}

	}

	ID = '\0';

}

void finding_maxmin(void) {
	strcpy(vlc, vlc);
	vlcf = atof(vlc);
	vlci = (int) (vlcf * 100.0);
	strcpy(alt, alt);
	altf = atof(alt);
	alti = (int) (altf * 100.0);
	if (ID == '0') {
		if (vlci > EPHmax_vlci)
			EPHmax_vlci = vlci;
		if (vlci < EPHmin_vlci)
			EPHmin_vlci = vlci;
		if (alti > EPHmax_alti)
			EPHmax_alti = alti;
		if (alti < EPHmin_alti)
			EPHmin_alti = alti;
	}
	else if (ID == '1') {
		if (vlci > PLmax_vlci)
			PLmax_vlci = vlci;
		if (vlci < PLmin_vlci)
			PLmin_vlci = vlci;
		if (alti > PLmax_alti)
			PLmax_alti = alti;
		if (alti < PLmin_alti)
			PLmin_alti = alti;
	}
}

double deg2rad(float deg) {
	double pi = 2 * acos(0.0);
	return deg * (pi / 180);
}

double rad2deg(float rad)
{
    double pi = 2 * acos(0.0);
    return (rad * 180 / pi);
}

float Cal_distGPS(float lat, float lng, float ground_latt, float ground_lngg) {
	dLat = deg2rad(lat - ground_latt); // deg2rad below
	dLon = deg2rad(lng - ground_lngg);
	a = sin(dLat / 2) * sin(dLat / 2)
			+ cos(deg2rad(lat)) * cos(deg2rad(ground_latt)) * sin(dLon / 2)
					* sin(dLon / 2);
	c = 2 * atan2(sqrt(a), sqrt(1 - a));
	d = R * c; // Distance in km
	distance = d * 1000;
	return distance;
}

float GetAngle(float lat, float lng, float ground_latt, float ground_lngg) {
	float Org_Enlem = lat;
	float Org_Lng = ground_lngg;

	float distance1 = Cal_distGPS(Org_Enlem, Org_Lng, lat, lng);
	float distance2 = Cal_distGPS(ground_latt, ground_lngg, Org_Enlem, Org_Lng);

	float alfa = rad2deg(atan2(distance1, distance2));
	if (lat == ground_latt && lng == ground_lngg) {
		alfa = 0;
		//cout << "degree: " << alfa << endl;
	} else if (lat == ground_latt && lng > ground_lngg) {
		alfa = 0;
		//cout << "degree: " << alfa << endl;
	} else if (lat == ground_latt && lng < ground_lngg) {
		alfa = 180;
		//cout << "degree: " << alfa << endl;
	} else if (lat > ground_latt && lng == ground_lngg) {
		alfa = 90;
		//cout << "degree: " << alfa << endl;
	} else if (lat > ground_latt && lng == ground_lngg) {
		alfa = 270;
		//cout << "degree: " << alfa << endl;
	} else if (lat > ground_latt && lng > ground_lngg) {
		//cout << "degree: " << alfa << endl;
	} else if (lat < ground_latt && lng > ground_lngg) {
		alfa = 180 - alfa;
		//cout << "degree: " << alfa << endl;
	} else if (lat > ground_latt && lng < ground_lngg) {
		alfa = 360 - alfa;
		//cout << "degree: " << alfa << endl;
	} else if (lat < ground_latt && lng < ground_lngg) {
		alfa = 180 + alfa;
		//cout << "degree: " << alfa << endl;
	}

	alfa -= 90.0;

	if (alfa < 0)
		alfa += 360;
	return alfa;
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
