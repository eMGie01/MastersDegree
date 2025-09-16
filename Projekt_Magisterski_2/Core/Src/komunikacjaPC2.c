/*
 * komunikacjaPC2.c
 *
 *  Created on: Mar 20, 2025
 *      Author: marek
 */

/* Includes */

#include "komunikacjaPC2.h"
#include "stm32l1xx_hal.h"

#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

/* Defines */
#define RX_BUFFER_SIZE 256

#define EEPROM_ADDR_HEADER     (FLASH_EEPROM_BASE)
#define EEPROM_ADDR_SETPOINT   (EEPROM_ADDR_HEADER + 4)
#define EEPROM_ADDR_MODE       (EEPROM_ADDR_SETPOINT + 4)
#define EEPROM_ADDR_ONOFF      (EEPROM_ADDR_MODE + 4)
#define EEPROM_ADDR_LOGGING    (EEPROM_ADDR_ONOFF + 4)
#define EEPROM_ADDR_BUF_SIZE   (EEPROM_ADDR_LOGGING + 4)
#define EEPROM_ADDR_SPEED      (EEPROM_ADDR_BUF_SIZE + 4)
#define EEPROM_ADDR_DATA43     (EEPROM_ADDR_SPEED + 4)
#define EEPROM_ADDR_DATA47     (EEPROM_ADDR_DATA43 + 4)
#define EEPROM_ADDR_DATA51     (EEPROM_ADDR_DATA47 + 4)
#define EEPROM_ADDR_DATA55     (EEPROM_ADDR_DATA51 + 4)
#define EEPROM_ADDR_DATA59     (EEPROM_ADDR_DATA55 + 4)

/* Variables */

		uint8_t 	uart_byte;
static 	uint8_t 	rx_buffer[RX_BUFFER_SIZE];
		uint16_t 	uart_idx = 0;

/* Functions */

void myprintf( const char *fmt, ...) {

    static char sd_buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(sd_buffer, sizeof(sd_buffer), fmt, args);
    va_end(args);
    int len = strlen(sd_buffer);
    HAL_UART_Transmit(&huart2, (uint8_t*)sd_buffer, len, -1);
}

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart) {

    if (huart->Instance == USART2) {

    	rx_buffer[uart_idx++] = uart_byte;
    	if (uart_idx >= RX_BUFFER_SIZE) uart_idx = 0;
        HAL_UART_Receive_IT(&huart2, &uart_byte, 1);
    }
}

void createDummyData( DataStruct *dummy) {

//	dummy->header = 0x72;
//	dummy->length = 0x26;
//	uint8_t 	trigger_pulse = 'N';
//	float wire_res = 0.0;

//	setSetpoint(0.7);
//	setCurrent();
//	setBatteryVoltage();
//	setParameter();
//	setDacOutput(0.000);
//	setMode('C');
//	setOnOff(0);
//	setLogging(0);
//	setLoggingSpeed(100);
//	setLoggingBufferSize(350);
//
//	memcpy( &dummy->data[43], &wire_res, sizeof(float));
//	float value = 0.1;
//	memcpy( &dummy->data[47], &value, sizeof(float));
//	memcpy( &dummy->data[51], &value, sizeof(float));
//	memcpy( &dummy->data[55], &value, sizeof(float));

	EEPROM_Load_Partial(dummy);

//	memcpy( &dummy->data[43], &trigger_pulse, sizeof(uint8_t));
//	float value = 10.0;
//	memcpy( &dummy->data[44], &value, sizeof(float)); // reference_level / V1
//	value = 5.0;
//	memcpy( &dummy->data[48], &value, sizeof(float)); // amplitude       / V2
//	value = 3.0;
//	memcpy( &dummy->data[52], &value, sizeof(float)); // frequency   	 / t1
//	memcpy( &dummy->data[56], &value, sizeof(float)); // 				 / t2
//	value = 7.0;
//	memcpy( &dummy->data[60], &value, sizeof(float)); //				 / t3
//	value = 0.1;
//	memcpy( &dummy->data[64], &value, sizeof(float)); // dt
	calculateCrc( dummy);
}

void calculateCrc( DataStruct *dummy) {

	dummy->crc = (uint32_t)dummy->header;
	dummy->crc += (uint32_t)dummy->length;
	for (int i=0; i<sizeof(dummy->data); i++) {

		dummy->crc += (uint32_t)dummy->data[i];
	}
}

void sendStructure( DataStruct *data) {

	//HAL_UART_Transmit(&huart2, (uint8_t*)&data->header, sizeof(data->header), -1);
	//HAL_UART_Transmit(&huart2, (uint8_t*)&data->length, sizeof(data->length), -1);
	//HAL_UART_Transmit(&huart2, (uint8_t*)&data->data, sizeof(data->data), -1);
	//HAL_UART_Transmit(&huart2, (uint8_t*)&data->crc, sizeof(data->crc), -1);
}

void checkForData(DataStruct *data) {

	static uint8_t message_start_idx = 0;
    static uint16_t read_idx = 0;
    static uint8_t buffer[37];
    static uint8_t buf_pos = 0;
    static uint8_t message_flag = 0;
    static uint8_t length = 0;
    static uint32_t crc = 0;
    while (read_idx != uart_idx) {
        uint8_t byte_ = rx_buffer[read_idx];

        if (message_flag == 0) {
            if (byte_ == 0x77) {
                message_flag = 1;
                buffer[0] = byte_;
                buf_pos = 1;
                crc = byte_;
                message_start_idx = read_idx;
            }
        } else {
            buffer[buf_pos++] = byte_;

            if (buf_pos == 2) {
                length = byte_;
            }
            if (buf_pos <= 33) {
                crc += byte_;
            }
            if (buf_pos == 36) {
                message_flag = 0;
                uint32_t rx_crc;
                memcpy(&rx_crc, &buffer[buf_pos - 3], sizeof(uint32_t));
                if (crc == rx_crc) {

                    saveDataToStruct(message_start_idx, length, data);
                    memcpy(&data->crc, &rx_crc, sizeof(float));
                }
                buf_pos = 0;
                crc = 0;
            }
        }
        read_idx++;
        if (read_idx >= RX_BUFFER_SIZE) read_idx = 0;
    }
}

void saveDataToStruct( uint16_t start_msg_idx, uint8_t length, DataStruct *data) {

	myprintf("I am in saving Data process...\r\n");

	memcpy(&data->header,   &rx_buffer[start_msg_idx],      sizeof(uint8_t));
	setSetpoint(*(float*)&rx_buffer[start_msg_idx + 2]);
	setMode(*(uint8_t*)&rx_buffer[start_msg_idx + 6]);
	setOnOff(*(uint8_t*)&rx_buffer[start_msg_idx + 7]);
	setLogging(*(uint8_t*)&rx_buffer[start_msg_idx + 8]);
	setLoggingBufferSize(*(uint16_t*)&rx_buffer[start_msg_idx + 9]);
	setLoggingSpeed(*(uint16_t*)&rx_buffer[start_msg_idx + 11]);

	memcpy(&data->data[43], &rx_buffer[start_msg_idx + 13], sizeof(float));
	memcpy(&data->data[47],  &rx_buffer[start_msg_idx + 17],  sizeof(float));
	memcpy(&data->data[51],  &rx_buffer[start_msg_idx + 21],  sizeof(float));
	memcpy(&data->data[55],  &rx_buffer[start_msg_idx + 25],  sizeof(float));
	memcpy(&data->data[59], &rx_buffer[start_msg_idx + 29], sizeof(float));

	EEPROM_Save_Partial(data);
}

void EEPROM_Save_Partial(DataStruct* data) {

	float value = 0;
    HAL_FLASHEx_DATAEEPROM_Unlock();

    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_HEADER, data->header);
    value = getSetpoint();
    uint32_t value_u32 = *(uint32_t*)&value;
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_SETPOINT, value_u32);
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_MODE, getMode());
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_ONOFF, getOnOff());
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_LOGGING, getLogging());
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_BUF_SIZE,
    		getLoggingBufferSize());
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_SPEED, getLoggingSpeed());
    value = *(float*)&data->data[43];
    value_u32 = *(uint32_t*)&value;
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_DATA43, value_u32);
    value = *(float*)&data->data[47];
    value_u32 = *(uint32_t*)&value;
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_DATA47, value_u32);
    value = *(float*)&data->data[51];
    value_u32 = *(uint32_t*)&value;
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_DATA51, value_u32);
    value = *(float*)&data->data[55];
    value_u32 = *(uint32_t*)&value;
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_DATA55, value_u32);
    value = *(float*)&data->data[59];
    value_u32 = *(uint32_t*)&value;
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_ADDR_DATA59, value_u32);

    HAL_FLASHEx_DATAEEPROM_Lock();
}

void EEPROM_Load_Partial(DataStruct* data) {

	uint32_t value_u32 = 0;
	float value = 0.0;

    data->header = *(uint8_t*)EEPROM_ADDR_HEADER;

    value_u32 = *(uint32_t*)EEPROM_ADDR_SETPOINT;
    value = *(float*)&value_u32;
    setSetpoint(value);

    setMode(*(uint8_t*)EEPROM_ADDR_MODE);
    setOnOff(*(uint8_t*)EEPROM_ADDR_ONOFF);
    setLogging(*(uint8_t*)EEPROM_ADDR_LOGGING);
    setLoggingBufferSize(*(uint16_t*)EEPROM_ADDR_BUF_SIZE);
    setLoggingSpeed(*(uint16_t*)EEPROM_ADDR_SPEED);

    value_u32 = *(uint32_t*)EEPROM_ADDR_DATA43;
    value = *(float*)&value_u32;
    memcpy(&data->data[43], &value, sizeof(float));
    value_u32 = *(uint32_t*)EEPROM_ADDR_DATA47;
    value = *(float*)&value_u32;
    memcpy(&data->data[47], &value, sizeof(float));
	value_u32 = *(uint32_t*)EEPROM_ADDR_DATA51;
    value = *(float*)&value_u32;
    memcpy(&data->data[51], &value, sizeof(float));
	value_u32 = *(uint32_t*)EEPROM_ADDR_DATA55;
    value = *(float*)&value_u32;
    memcpy(&data->data[55], &value, sizeof(float));
	value_u32 = *(uint32_t*)EEPROM_ADDR_DATA59;
    value = *(float*)&value_u32;
    memcpy(&data->data[59], &value, sizeof(float));
}


