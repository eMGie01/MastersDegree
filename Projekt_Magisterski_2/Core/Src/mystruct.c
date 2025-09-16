/*
 * mystruct.c
 *
 *  Created on: Mar 20, 2025
 *      Author: marek
 */

#include "mystruct.h"
#include "gpio.h"
#include "spi.h"
#include "komunikacjaPC2.h"
#include "sd.h"
#include "regulator.h"

#include <string.h>
#include <math.h>

#define V_IN_MIN      10.5f
#define V_IN_MAX      15.0f
#define V_REF 			2.5
#define V_MAX_DAC 		3.999
#define RESOLUTION_24 	0x7FFFFE

int32_t current_offset = 0;
float qlmb = 0;

DataStruct myData;

float getSetpoint(void) {
 	return *(float*)&myData.data[0];
}

void setSetpoint(float setpoint) {

	if (getMode() == 'C') {

		if (setpoint > 15.0f)
			setpoint = 15.0f;
		else if (setpoint < 0)
			setpoint = 0.0f;
	}
	else if (getMode() == 'P') {

		if (setpoint > 180.0f)
			setpoint = 180.0f;
		else if (setpoint < 0.0f)
			setpoint = 0.0f;
	}
	else if (getMode() == 'R') {

		if (setpoint < 0.8f)
			setpoint = 0.8f;
		else if (setpoint > 80.0f)
			setpoint = 80.0f;
	}

	setpoint = roundf(setpoint * 1000.0f) / 1000.0f;
	memcpy(&myData.data[0], &setpoint, sizeof(float));
}

float setQlmb(void) {

	Qlmb += getCurrent() * 0.05f;

	if (getOnOff() != 1)
		Qlmb = 0;

	memcpy(&myData.data[63], &Qlmb, sizeof(float));
}

float getCurrent(void) {
	return *(float*)&myData.data[8];
}

void setCurrent(void) {

	uint8_t Voltage[3];
	uint8_t readVoltage = 0b01000001;
	HAL_GPIO_WritePin(CS_2_GPIO_Port, CS_2_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &readVoltage, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, &Voltage[0],  3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_2_GPIO_Port, CS_2_Pin, GPIO_PIN_SET);
	int32_t raw_data = (Voltage[0] << 16) | (Voltage[1] << 8) | Voltage[2];

	float current = 0.0;
	float setpoint = 0.0;
	if (getMode() == 'P')
		setpoint = getSetpoint() / getBatteryVoltage();
	else if (getMode() == 'R')
		setpoint = getBatteryVoltage() / getSetpoint();
	else
		setpoint = getSetpoint();

	if (setpoint >= 0.0f && setpoint <= 5.0) {
		current = ((float)raw_data*(4.74f/294092.3333f) + (0.17-26377.6667*(4.74f/294092.3333f)));
	}
	else if (setpoint > 5.0f && setpoint <= 10.0) {
		current = ((float)raw_data*(5.086667f/418925) + (5.06667f-333558.333f*(5.086667f/418925)));
	}
	else if (setpoint > 10.0f && setpoint <= 15.0) {
		current = ((float)raw_data*(3.44f/461250.333f) + (11.04f-855380.333f*(3.44f/461250.333f)));
	}

	float value = 0.0;

	if (getMode() == 'C') {

		value = getSetpoint();
	}
	else if (getMode() == 'R') {

		value = getBatteryVoltage() / getSetpoint();
	}
	else if (getMode() == 'P') {

		value = getSetpoint() / getBatteryVoltage();
	}

	if (value <= 5.0f) {

		HAL_GPIO_WritePin(RES_1_GPIO_Port, RES_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RES_2_GPIO_Port, RES_2_Pin, GPIO_PIN_RESET);
	}
	else if (value > 5.0f && value <= 11.0f) {

		HAL_GPIO_WritePin(RES_1_GPIO_Port, RES_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RES_2_GPIO_Port, RES_2_Pin, GPIO_PIN_RESET);
	}
	else if (value > 11.0f && value <= 15.0f) {

		HAL_GPIO_WritePin(RES_1_GPIO_Port, RES_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RES_2_GPIO_Port, RES_2_Pin, GPIO_PIN_SET);
	}

	if (getOnOff() != 1)
		current = 0.0f;

	qlmb += current*0.05f;

	memcpy(&myData.data[8], &current, sizeof(float));
}

float getBatteryVoltage(void) {
	return *(float*)&myData.data[12];
}

void setBatteryVoltage(void) {

	static const float a = 1.4978/2913086;
	static const float b = 11.506 - (1838100*a);
	uint8_t Voltage[3];
	uint8_t readVoltage = 0b01000001;
	HAL_GPIO_WritePin(CS_1_GPIO_Port, CS_1_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &readVoltage, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, &Voltage[0],  3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_1_GPIO_Port, CS_1_Pin, GPIO_PIN_SET);
	int32_t raw_data = (Voltage[0] << 16) | (Voltage[1] << 8) | Voltage[2];
	float battery_voltage = 0;
	float wire_resistance = *(float*)&myData.data[43];

	if (getOnOff() != 0) {
		battery_voltage = (((float)raw_data * a) + b) + getCurrent()*wire_resistance;
	}
	else {
		battery_voltage = (((float)raw_data * a) + b);
	}

    memcpy(&myData.data[12], &battery_voltage, sizeof(float));
}

float getDacOutput(void) {
	return *(float*)&myData.data[16];
}

void setDacOutput(uint16_t dac_output) {

	uint8_t dac_data[2] = { dac_output >> 8, dac_output & 0xFF };
	HAL_GPIO_WritePin(CS_3_GPIO_Port, CS_3_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, dac_data, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_3_GPIO_Port, CS_3_Pin, GPIO_PIN_SET);
	float DAC_out = ((float)dac_output / 0xFFFE) * V_MAX_DAC;
	memcpy(&myData.data[16], &DAC_out, sizeof(float));
}

float getParameter(void) {
	return *(float*)&myData.data[4];
}

void setParameter(void) {

	float parameter = 0.0;
	if (getMode() == 'C')
		parameter = getCurrent();
	if (getMode() == 'R')
		parameter = getBatteryVoltage() / getCurrent();
	if (getMode() == 'P')
		parameter = getBatteryVoltage() * getCurrent();

	memcpy(&myData.data[4], &parameter, sizeof(float));
}

uint8_t getMode(void) {
	return *(uint8_t*)&myData.data[36];
}

void setMode(uint8_t mode) {
	memcpy(&myData.data[36], &mode, sizeof(uint8_t));
}

uint8_t getOnOff(void) {
	return *(uint8_t*)&myData.data[37];
}

void setOnOff(uint8_t onOff) {

	if (onOff == 0)
		setDacOutput(0);
	memcpy(&myData.data[37], &onOff, sizeof(uint8_t));
}

uint8_t getLogging(void) {
	return *(uint8_t*)&myData.data[38];
}

void setLogging(uint8_t logging) {

	if (logging != 1)
		prev_logging = 0;

	memcpy(&myData.data[38], &logging, sizeof(uint8_t));
}

uint16_t getLoggingBufferSize(void) {
	return *(uint16_t*)&myData.data[39];
}

void setLoggingBufferSize(uint16_t buffer_size) {
	memcpy(&myData.data[39], &buffer_size, sizeof(uint16_t));
}

uint16_t getLoggingSpeed(void) {
	return *(uint16_t*)&myData.data[41];
}

void setLoggingSpeed(uint16_t speed) {
	memcpy(&myData.data[41], &speed, sizeof(uint16_t));
}

void measureTemp_1(void) {

	wire_reset_1();
	wire_write_1(0xcc);
	wire_write_1(0x44);
}

void setTemp_1(void) {

	  wire_reset_1();
	  wire_write_1(0xcc);
	  wire_write_1(0xbe);
	  int i;
	  uint8_t scratchpad[9];
	  for (i = 0; i < 9; i++) {
		  scratchpad[i] = wire_read_1();
	  }
	  uint8_t higher = scratchpad[1];
	  uint8_t lower = scratchpad[0];
	  uint16_t combined = ((uint16_t)higher << 8) | lower;
	  float temperature = (float)combined / 16.0;

	  if (temperature < 4000)
		  memcpy(&myData.data[20], &temperature, sizeof(float));
}

float getTemp_1(void) {
	return *(float*)&myData.data[20];
}

void measureTemp_2(void) {

	wire_reset_2();
	wire_write_2(0xcc);
	wire_write_2(0x44);
}

void setTemp_2(void) {

	  wire_reset_2();
	  wire_write_2(0xcc);
	  wire_write_2(0xbe);
	  int i;
	  uint8_t scratchpad[9];
	  for (i = 0; i < 9; i++) {
		  scratchpad[i] = wire_read_2();
	  }
	  uint8_t higher = scratchpad[1];
	  uint8_t lower = scratchpad[0];
	  uint16_t combined = ((uint16_t)higher << 8) | lower;
	  float temperature = (float)combined / 16.0;\

	  if (temperature < 4000)
		  memcpy(&myData.data[24], &temperature, sizeof(float));
}

float getTemp_2(void) {
	return *(float*)&myData.data[24];
}

void measureTemp_3(void) {

	wire_reset_3();
	wire_write_3(0xcc);
	wire_write_3(0x44);
}

void setTemp_3(void) {

	  wire_reset_3();
	  wire_write_3(0xcc);
	  wire_write_3(0xbe);
	  int i;
	  uint8_t scratchpad[9];
	  for (i = 0; i < 9; i++) {
		  scratchpad[i] = wire_read_3();
	  }
	  uint8_t higher = scratchpad[1];
	  uint8_t lower = scratchpad[0];
	  uint16_t combined = ((uint16_t)higher << 8) | lower;
	  float temperature = (float)combined / 16.0;

	  if (temperature < 2000)
		  memcpy(&myData.data[28], &temperature, sizeof(float));
}

float getTemp_3(void) {
	return *(float*)&myData.data[28];
}

void delay_us(uint32_t us) {
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  while (__HAL_TIM_GET_COUNTER(&htim6) < us) {}
}
HAL_StatusTypeDef wire_reset_1(void) {

  int rc;
  HAL_GPIO_WritePin(temp_1_GPIO_Port, temp_1_Pin, GPIO_PIN_RESET);
  delay_us(480);
  HAL_GPIO_WritePin(temp_1_GPIO_Port, temp_1_Pin, GPIO_PIN_SET);
  delay_us(70);
  rc = HAL_GPIO_ReadPin(temp_1_GPIO_Port, temp_1_Pin);
  delay_us(410);
  if (rc == 0)
    return HAL_OK;
  else
    return HAL_ERROR;
}
void write_bit_1(int value) {

  if (value) {
    HAL_GPIO_WritePin(temp_1_GPIO_Port, temp_1_Pin, GPIO_PIN_RESET);
    delay_us(6);
    HAL_GPIO_WritePin(temp_1_GPIO_Port, temp_1_Pin, GPIO_PIN_SET);
    delay_us(64);
  }
  else {
    HAL_GPIO_WritePin(temp_1_GPIO_Port, temp_1_Pin, GPIO_PIN_RESET);
    delay_us(60);
    HAL_GPIO_WritePin(temp_1_GPIO_Port, temp_1_Pin, GPIO_PIN_SET);
    delay_us(10);
  }
}
int read_bit_1(void) {

  int rc;
  HAL_GPIO_WritePin(temp_1_GPIO_Port, temp_1_Pin, GPIO_PIN_RESET);
  delay_us(6);
  HAL_GPIO_WritePin(temp_1_GPIO_Port, temp_1_Pin, GPIO_PIN_SET);
  delay_us(9);
  rc = HAL_GPIO_ReadPin(temp_1_GPIO_Port, temp_1_Pin);
  delay_us(55);
  return rc;
}

void wire_write_1(uint8_t byte) {

  int i;
  for (i = 0; i < 8; i++) {
    write_bit_1(byte & 0x01);
    byte >>= 1;
  }
}

uint8_t wire_read_1(void) {

  uint8_t value = 0;
  int i;
  for (i = 0; i < 8; i++) {
    value >>= 1;
    if (read_bit_1())
      value |= 0x80;
  }
  return value;
}

/* TEMP_2 */
HAL_StatusTypeDef wire_reset_2(void) {

  int rc;
  HAL_GPIO_WritePin(temp_2_GPIO_Port, temp_2_Pin, GPIO_PIN_RESET);
  delay_us(480);
  HAL_GPIO_WritePin(temp_2_GPIO_Port, temp_2_Pin, GPIO_PIN_SET);
  delay_us(70);
  rc = HAL_GPIO_ReadPin(temp_2_GPIO_Port, temp_2_Pin);
  delay_us(410);
  if (rc == 0)
    return HAL_OK;
  else
    return HAL_ERROR;
}

void write_bit_2(int value) {

  if (value) {
    HAL_GPIO_WritePin(temp_2_GPIO_Port, temp_2_Pin, GPIO_PIN_RESET);
    delay_us(6);
    HAL_GPIO_WritePin(temp_2_GPIO_Port, temp_2_Pin, GPIO_PIN_SET);
    delay_us(64);
  }
  else {
    HAL_GPIO_WritePin(temp_2_GPIO_Port, temp_2_Pin, GPIO_PIN_RESET);
    delay_us(60);
    HAL_GPIO_WritePin(temp_2_GPIO_Port, temp_2_Pin, GPIO_PIN_SET);
    delay_us(10);
  }
}

int read_bit_2(void) {

  int rc;
  HAL_GPIO_WritePin(temp_2_GPIO_Port, temp_2_Pin, GPIO_PIN_RESET);
  delay_us(6);
  HAL_GPIO_WritePin(temp_2_GPIO_Port, temp_2_Pin, GPIO_PIN_SET);
  delay_us(9);
  rc = HAL_GPIO_ReadPin(temp_2_GPIO_Port, temp_2_Pin);
  delay_us(55);
  return rc;
}

void wire_write_2(uint8_t byte) {

  int i;
  for (i = 0; i < 8; i++) {
    write_bit_2(byte & 0x01);
    byte >>= 1;
  }
}

uint8_t wire_read_2(void) {

  uint8_t value = 0;
  int i;
  for (i = 0; i < 8; i++) {
    value >>= 1;
    if (read_bit_2())
      value |= 0x80;
  }
  return value;
}

/* TEMP_3 */
HAL_StatusTypeDef wire_reset_3(void) {

  int rc;
  HAL_GPIO_WritePin(temp_3_GPIO_Port, temp_3_Pin, GPIO_PIN_RESET);
  delay_us(480);
  HAL_GPIO_WritePin(temp_3_GPIO_Port, temp_3_Pin, GPIO_PIN_SET);
  delay_us(70);
  rc = HAL_GPIO_ReadPin(temp_3_GPIO_Port, temp_3_Pin);
  delay_us(410);
  if (rc == 0)
    return HAL_OK;
  else
    return HAL_ERROR;
}

void write_bit_3(int value) {

  if (value) {
    HAL_GPIO_WritePin(temp_3_GPIO_Port, temp_3_Pin, GPIO_PIN_RESET);
    delay_us(6);
    HAL_GPIO_WritePin(temp_3_GPIO_Port, temp_3_Pin, GPIO_PIN_SET);
    delay_us(64);
  }
  else {
    HAL_GPIO_WritePin(temp_3_GPIO_Port, temp_3_Pin, GPIO_PIN_RESET);
    delay_us(60);
    HAL_GPIO_WritePin(temp_3_GPIO_Port, temp_3_Pin, GPIO_PIN_SET);
    delay_us(10);
  }
}

int read_bit_3(void) {

  int rc;
  HAL_GPIO_WritePin(temp_3_GPIO_Port, temp_3_Pin, GPIO_PIN_RESET);
  delay_us(6);
  HAL_GPIO_WritePin(temp_3_GPIO_Port, temp_3_Pin, GPIO_PIN_SET);
  delay_us(9);
  rc = HAL_GPIO_ReadPin(temp_3_GPIO_Port, temp_3_Pin);
  delay_us(55);
  return rc;
}

void wire_write_3(uint8_t byte) {

  int i;
  for (i = 0; i < 8; i++) {
    write_bit_3(byte & 0x01);
    byte >>= 1;
  }
}

uint8_t wire_read_3(void) {

  uint8_t value = 0;
  int i;
  for (i = 0; i < 8; i++) {
    value >>= 1;
    if (read_bit_3())
      value |= 0x80;
  }
  return value;
}

//uint8_t * trigger_function = (uint8_t*)&myData.data[43];
//float * reference_level = (float*)&myData.data[44];
//float * amplitude       = (float*)&myData.data[48];
//float * frequency		= (float*)&myData.data[52];
//float * V1 = (float*)&myData.data[44];
//float * V2 = (float*)&myData.data[48];
//float * t1 = (float*)&myData.data[52];
//float * t2 = (float*)&myData.data[56];
//float * t3 = (float*)&myData.data[60];

