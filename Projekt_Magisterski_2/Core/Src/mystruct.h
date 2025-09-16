/*
 * mystruct.h
 *
 *  Created on: Mar 20, 2025
 *      Author: marek
 */

#ifndef MYSTRUCT_H_
#define MYSTRUCT_H_

#include <stdint.h>

#include "tim.h"

extern int32_t current_offset;
extern float qlmb;

typedef struct {

	uint8_t header;
	uint8_t length;
	uint8_t data[67];
	uint32_t crc;
} DataStruct;

extern DataStruct myData;

float 	getSetpoint(void);
void 	setSetpoint(float setpoint);
float 	getCurrent(void);
void 	setCurrent(void);
float 	getBatteryVoltage(void);
void 	setBatteryVoltage(void);
void 	setADCResolution(void);
float 	getDacOutput(void);
void 	setDacOutput(uint16_t dac_output);
float 	getParameter(void);
uint8_t readConfig0(void);
void 	setParameter(void);
uint8_t getMode(void);
void 	setMode(uint8_t mode);
uint8_t getOnOff(void);
void 	setOnOff(uint8_t onOff);
uint8_t getLogging(void);
void 	setLogging(uint8_t logging);
uint16_t getLoggingBufferSize(void);
void 	setLoggingBufferSize(uint16_t buffer_size);
uint16_t getLoggingSpeed(void);
void 	setLoggingSpeed(uint16_t speed);

void measureTemp_1(void);
void setTemp_1(void);
float getTemp_1(void);

void measureTemp_2(void);
void setTemp_2(void);
float getTemp_2(void);

void measureTemp_3(void);
void setTemp_3(void);
float getTemp_3(void);

void delay_us(uint32_t us);

HAL_StatusTypeDef wire_reset_1(void);
void write_bit_1(int value);
int read_bit_1(void);
void wire_write_1(uint8_t byte);
uint8_t wire_read_1(void);

HAL_StatusTypeDef wire_reset_2(void);
void write_bit_2(int value);
int read_bit_2(void);
void wire_write_2(uint8_t byte);
uint8_t wire_read_2(void);

HAL_StatusTypeDef wire_reset_3(void);
void write_bit_3(int value);
int read_bit_3(void);
void wire_write_3(uint8_t byte);
uint8_t wire_read_3(void);

#define temp_1 *(float*)&myData.data[20]
#define temp_2 *(float*)&myData.data[24]
#define temp_3 *(float*)&myData.data[28]
#define temp_4 *(float*)&myData.data[32]


#define wire_Res *(float*)myData.data[43]
#define Kp *(float*)&myData.data[47]
#define Ki *(float*)&myData.data[51]
#define Kd *(float*)&myData.data[55]
#define timeStop *(float*)&myData.data[59]

#endif /* MYSTRUCT_H_ */
