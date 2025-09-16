/*
 * interface.h
 *
 *  Created on: Mar 20, 2025
 *      Author: marek
 */

#ifndef SRC_INTERFACE_H_
#define SRC_INTERFACE_H_

#include <stdint.h>

void lcdEnablePulse(void);
void lcdSend4Bits(uint8_t data);
void lcdSendCmnd(uint8_t cmnd);
void lcdInit(void);
void lcdSendData(uint8_t data);
void lcdSendString(char *str);
void lcdDisplayMeasurement(void);
void lcdDisplaySetpoint(void);
void lcdDisplayMode(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void lcdCheckForUpdate(void);

#endif /* SRC_INTERFACE_H_ */
