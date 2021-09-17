/*
 * MAGNETO.c
 *
 *  Created on: 23 Tem 2021
 *      Author: Beytu
 */
#include "MAGNETO.h"
#include "math.h"
#include "stdio.h"
#include "MPU-6050.h"
#include "nextion.h"  //*************

#define PI 3.1415926535897932

uint8_t k = 0;
//uint8_t kn[2] = {0x0D, 0};  	// read
//uint8_t setr[2] = {0x0B, 0x01}; // write
//uint8_t cntr[2] = {0x09, 0x1D}; // write
//uint8_t drdy[2] = {0x06, 0};  	// read
//uint8_t data[6] = {0};  		// read
//uint8_t data_ready = 0;

int16_t Xm = 0, Ym = 0, Zm = 0;  // uint16_t
float compass_n, compass;
float roll = .0, pitch = .0;
float Ax, Ay, Az;

uint8_t enter = 0;
extern int len;
extern uint8_t cmd_nxt[3];

uint8_t command[4] = {0x70, 0xA0, 0x00, 0X03};
//uint8_t command[4] = {0x70, 0x20, 0x00, 0X03};
uint8_t data[6] = {0};
uint8_t countm = 0;

int16_t Xmax, Xmin, Ymax, Ymin;
float Xsf, Ysf, Xoff, Yoff;

void magneto_Init(uint8_t confg_c) {

//	HAL_I2C_Mem_Write(&hi2c1, MGN_WRITE_ADD, setr[0], 1, &setr[1], 1, 100);
//	HAL_I2C_Mem_Write(&hi2c1, MGN_WRITE_ADD, cntr[0], 1, &cntr[1], 1, 100);


	HAL_I2C_Mem_Write(&hi2c1, MGN_WRITE_ADD, 0x00, 1, &command[0], 1, 100);
	HAL_I2C_Mem_Write(&hi2c1, MGN_WRITE_ADD, 0x01, 1, &command[1], 1, 100);
	HAL_I2C_Mem_Write(&hi2c1, MGN_WRITE_ADD, 0x02, 1, &command[2], 1, 100);
	HAL_Delay(100);
	uint32_t full = 0;
	NXT_SEND_STR("page page0");
	for (uint8_t i = 0; i < confg_c; i++) {
		Read_mgn();
		HAL_Delay(70);
		if (enter == 0) {
			Ymin = Ym;
			enter++;
		}
		if (Xm < Xmin)
			Xmin = Xm;
		if (Xm > Xmax)
			Xmax = Xm;
		if (Ym < Ymin)
			Ymin = Ym;
		if (Ym > Ymax)
			Ymax = Ym;
		full = ((float)i / (float)confg_c) * 100;
		NXT_SEND_VAL("j0.val", full);
	}
	NXT_SEND_STR("page page1");
	Xsf = (float) (Ymax - Ymin) / (float) (Xmax - Xmin);
	Ysf = (float) (Xmax - Xmin) / (float) (Ymax - Ymin);
	if (Xsf < 1)
		Xsf = 1.0;
	else if (Ysf < 1)
		Ysf = 1.0;

	Xoff = ((Xmax - Xmin) / 2 - Xmax) * Xsf;
	Yoff = ((Ymax - Ymin) / 2 - Ymax) * Ysf;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
}

void Read_mgn(void) {
	HAL_I2C_Mem_Read(&hi2c1, MGN_READ_ADD, 0x03, 1, data, 6, 100);
	HAL_I2C_Master_Transmit(&hi2c1, MGN_WRITE_ADD, &command[3], 1, 100);

//	HAL_I2C_Mem_Read(&hi2c1, MGN_READ_ADD, 0x06, 1, &data_ready, 6, 100);
//	if((data_ready & 0x01) == 1){
//		HAL_I2C_Mem_Read(&hi2c1, MGN_READ_ADD, 0x00, 1, data, 6, 100);
//
//		Xm = (int16_t)(data[1] << 8) | data[0];
//		Ym = (int16_t)(data[3] << 8) | data[2];
//		Zm = (int16_t)(data[5] << 8) | data[4];
//	}

	Xm = (int16_t)(data[0] << 8) | data[1];
	Ym = (int16_t)(data[4] << 8) | data[5];
	Zm = (int16_t)(data[2] << 8) | data[3];

//	if (Xm > 32767)
//		Xm = 0xFFFF - Xm + 1;
//	if (Zm > 32767)
//		Zm = 0xFFFF - Zm + 1;
//	if (Ym > 32767)
//		Ym = 0xFFFF - Ym + 1;

//	Xm = (int16_t)(data[1] << 8) | data[0];
//	Ym = (int16_t)(data[5] << 8) | data[4];
//	Zm = (int16_t)(data[3] << 8) | data[2];
}

float Read_compass(void) {
	Read_mgn();

	Az = MPU6050_AccZ();
	Ay = MPU6050_AccX();
	Ax = MPU6050_AccY();

	roll = atan2(Ay , Az) * 180 / PI;
	pitch = atan2((- Ax) , sqrt(Ay * Ay + Az * Az)) * 180 / PI;

	NXT_SEND_SVAL("n0.val", (int)roll);
	NXT_SEND_SVAL("n1.val", (int)pitch);

//	roll = deg2rad(roll);
//	pitch = deg2rad(pitch);

//	Xm = (Xm * (1 - pow(Ax , 2))) - (Ym * Ax * Ay) - (Zm * Ax * sqrt(1 - pow(Ax , 2) - pow(Ay , 2)));
//	Ym = (Ym * sqrt(1 - pow(Ax , 2) - pow(Ay , 2))) - Zm * Ay;

//	Xm = Xm * cos(pitch) + Ym * sin(roll) * sin(pitch) - Zm * cos(roll) * sin(pitch);
//	Ym = Ym * cos(roll) + Zm * sin(roll);

	Xm = Xm * Xsf + Xoff;  // ********************
	Ym = Ym * Ysf + Yoff;



	if (Xm == 0 && Ym < 0)
		compass_n = 90.0;
	else if (Xm == 0 && Ym > 0)
		compass_n = 270.0;
	else {
		compass_n = (atan2f(Ym, Xm) * (180.0 / PI));
	}
//	if (Ym == 0 && Xm > 0)
//			compass_n = 0.0;
//		else if (Ym == 0 && Xm < 0)
//			compass_n = 180.0;
//		else if (Ym > 0){
//			compass_n = 90.0 - (atanf(Ym/Xm)) * 180.0 / PI;
//			//compass_n = (atan2f(Ym, Xm) * (180.0 / PI));
//		}
//		else if (Ym < 0){
//					compass_n = 270.0 - (atanf(Ym/Xm)) * 180.0 / PI;
//					//compass_n = (atan2f(Ym, Xm) * (180.0 / PI));
//				}
	compass_n = compass_n + MGN_density + MGN_default_Angle;
	if (compass_n < 0)
		compass = compass_n + 360;
	else
		compass = compass_n;
	return compass;
}

/*double deg2rad(float deg) {
	double pi = 2 * acos(0.0);
	return deg * (pi / 180);
}*/

