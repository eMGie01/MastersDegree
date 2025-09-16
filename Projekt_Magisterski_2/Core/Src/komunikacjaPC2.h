/*
 * komunikacjaPC2.h
 *
 *  Created on: Mar 20, 2025
 *      Author: marek
 */

#ifndef SRC_KOMUNIKACJAPC2_H_
#define SRC_KOMUNIKACJAPC2_H_

/* Includes */
#include "mystruct.h"
#include "usart.h"

/* Variables */
extern uint8_t uart_byte;

/* Functions */
void myprintf( const char *fmt, ...);
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart);
void createDummyData( DataStruct *dummy);
void calculateCrc( DataStruct *dummy);
void sendStructure( DataStruct *data);
void checkForData( DataStruct *data);
void saveDataToStruct( uint16_t start_msg_idx, uint8_t length, DataStruct *data);
void EEPROM_Save_Partial(DataStruct* data);
void EEPROM_Load_Partial(DataStruct* data);

#endif /* SRC_KOMUNIKACJAPC2_H_ */
