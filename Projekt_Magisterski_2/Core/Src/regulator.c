/*
 * regulator.c
 *
 *  Created on: Mar 22, 2025
 *      Author: marek
 */
#include "regulator.h"
#include "mystruct.h"
#include "spi.h"
#include "komunikacjaPC2.h"
#include <math.h>
#include <string.h>

#define PI 3.141592653589793

//#define Kp 0.6		// dla okresu 50ms: 0.6
//#define Ki 1.8		// dla okresu 50ms: 1.8
//#define Kd 0.012		// dla okresu 50ms: 0.012

#define VREF 2.5
#define CURRENT_MAX 15.0

void regulatorPI(void) {

	static float integral = 0.0;
	static float prev_error = 0.0;
	float setpoint = 0.0;

	if (getMode() == 'P')
		setpoint = getSetpoint() / getBatteryVoltage();
	else if (getMode() == 'R')
		setpoint = getBatteryVoltage() / getSetpoint();
	else
		setpoint = getSetpoint();

	if (getOnOff() != 0) {

		float error = setpoint - getCurrent();
		float derivative = (error - prev_error) / 0.05f;
		prev_error = error;

		float provisional_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
		provisional_output *= compensate(getCurrent(), getTemp_1());
		if (provisional_output > 0.0f && provisional_output < 15.0f) {
			integral += error * 0.05f;
		}

		float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
		output *= compensate(getCurrent(), getTemp_1());
		if (output > 15.0f) output = 15.0f;
		if (output < 0.0f) output = 0.0f;
		uint16_t dac_output = (uint16_t)((output / 15.0) * 37683) + 27852;
		setDacOutput(dac_output);
	}
	else {

		integral = 0.0;
		prev_error = 0.0;
		uint16_t dac_output = 0;
		setDacOutput(dac_output);
	}
}

float compensate(float current, float temperature) {

    const float Vgs_table[3][16] = {
        {2.100, 2.266, 2.351, 2.420, 2.480, 2.534, 2.585, 2.632,
         2.678, 2.721, 2.763, 2.804, 2.843, 2.882, 2.919, 2.956},
        {2.002, 2.223, 2.321, 2.400, 2.466, 2.528, 2.585, 2.638,
         2.690, 2.738, 2.786, 2.831, 2.875, 2.918, 2.960, 3.002},
        {1.900, 2.176, 2.286, 2.372, 2.448, 2.517, 2.580, 2.639,
         2.696, 2.749, 2.802, 2.852, 2.900, 2.948, 2.994, 3.039}
    };

    if (current < 0.0f) 		current = 0.0f;
    if (current >= 15.0f) 		current = 14.999f;
    if (temperature < 25.0f) 	temperature = 25.0f;
    if (temperature > 175.0f) 	temperature = 175.0f;

    int i = (int)current;
    int temp_l_idx = 0;
    int temp_h_idx = 0;
    float corr_temp = 0.0f;
    if (temperature < 100) {
        temp_l_idx = 0;
        temp_h_idx = 1;
        corr_temp = (temperature - 25) / 75.0f;
    }
    else {
        temp_l_idx = 1;
        temp_h_idx = 2;
        corr_temp = (temperature - 100) / 75.0f;
    }

    float corr_25 = Vgs_table[0][i] + (current - (float)i) * (Vgs_table[0][i+1] - Vgs_table[0][i]);
    if (corr_25 < 0.001f)	return 1.0f;

    float corr_t_low = Vgs_table[temp_l_idx][i] + (current - (float)i) * (Vgs_table[temp_l_idx][i+1] - Vgs_table[temp_l_idx][i]);
    float corr_t_high = Vgs_table[temp_h_idx][i] + (current - (float)i) * (Vgs_table[temp_h_idx][i+1] - Vgs_table[temp_h_idx][i]);

    return (float)((corr_t_low + corr_temp * (corr_t_high - corr_t_low)) / corr_25);
}


void MCP3561T_1_Init(GPIO_TypeDef  *Port, uint16_t Pin) {

	  uint8_t config0_cmnd[2] = {0b01000110, 0b01100011};
	  uint8_t config1_cmnd[2] = {0b01001010, 0b00111100};
	  uint8_t config2_cmnd[2] = {0b01001110, 0b10001011}; // 110
	  uint8_t config3_cmnd[2] = {0b01010010, 0b11000000};
	  uint8_t MUX_cmnd[2] 	  = {0b01011010, 0b00001000}; // 0b00001000
	  uint8_t IRQ_cmnd[2] 	  = {0b01010110, 0b00000110};
	  uint8_t FAST_cmnd 	  =  0b01101000;

	  HAL_Delay(100);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &config0_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &config1_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &config2_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &config3_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &IRQ_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &MUX_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &FAST_cmnd, 1, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin,  GPIO_PIN_SET);

	  uint8_t byte = 0;
	  uint8_t tx_byte = {0b01000101};
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi2, &tx_byte, &byte, 1, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin,  GPIO_PIN_SET);

	  HAL_Delay(100);
}

void MCP3561T_2_Init(GPIO_TypeDef  *Port, uint16_t Pin) {

	  uint8_t config0_cmnd[2] = {0b01000110, 0b01100011};
	  uint8_t config1_cmnd[2] = {0b01001010, 0b00111100};
	  uint8_t config2_cmnd[2] = {0b01001110, 0b10001011}; // 110
	  uint8_t config3_cmnd[2] = {0b01010010, 0b11000010};
	  uint8_t MUX_cmnd[2] 	  = {0b01011010, 0b00000001}; // 0b00001000
	  uint8_t IRQ_cmnd[2] 	  = {0b01010110, 0b00000110};
	  uint8_t FAST_cmnd = 0b01101000;

	  HAL_Delay(100);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &config0_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &config1_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &config2_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &config3_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &IRQ_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &MUX_cmnd[0], 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, &FAST_cmnd, 1, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Port, Pin,  GPIO_PIN_SET);

	  HAL_Delay(50);

	  int32_t raw_data = 0;
	  for(int i=0; i<5; i++) {

		  uint8_t tx = 0b01000001;
		  uint8_t adc_read[3];
		  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
		  HAL_SPI_Transmit(&hspi2, &tx, 1, -1);
		  HAL_SPI_Receive(&hspi2, adc_read, 3, -1);
		  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
		  raw_data += ((int32_t)adc_read[0] << 16) | ((int32_t)adc_read[1] << 8) | (int32_t)adc_read[2];
		  HAL_Delay(50);
	  }
	  current_offset = raw_data / 5;
}
