/*
 * regulator.h
 *
 *  Created on: Mar 22, 2025
 *      Author: marek
 */

#ifndef SRC_REGULATOR_H_
#define SRC_REGULATOR_H_

#include "stdint.h"
#include "gpio.h"

void regulatorPI(void);
float compensate(float current, float temperature);
void updateSetpoint(uint8_t i, uint16_t *counter);
void MCP3561T_1_Init(GPIO_TypeDef  *Port, uint16_t Pin);
void MCP3561T_2_Init(GPIO_TypeDef  *Port, uint16_t Pin);

#endif /* SRC_REGULATOR_H_ */
