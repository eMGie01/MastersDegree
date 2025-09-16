/*
 * interface.c
 *
 *  Created on: Mar 20, 2025
 *      Author: marek
 */

/* Includes */

#include "interface.h"
#include "mystruct.h"
#include "gpio.h"

#include <stdio.h>

#define ON 	1
#define OFF 0

#define CC 0
#define CR 1
#define CP 2

typedef enum {

	DEFAULT = 0,
	TURN,
	MODE,
	LOGGING
} Mode_t;

volatile uint8_t lcd_update_flag = 0;
volatile uint8_t lcd_trigger_flag = 0;
volatile uint8_t lcd_disp_flag = 0;

Mode_t _display;

uint8_t lcd_menu_display = 0;
uint8_t lcd_menu_idx = 0;

void lcdEnablePulse(void) {

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
}

void lcdSend4Bits(uint8_t data) {

	HAL_GPIO_WritePin(DB4_GPIO_Port, DB4_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB5_GPIO_Port, DB5_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB6_GPIO_Port, DB6_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB7_GPIO_Port, DB7_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	lcdEnablePulse();
}


void lcdSendCmnd(uint8_t cmnd) {

	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
	lcdSend4Bits(cmnd >> 4);
	lcdSend4Bits(cmnd & 0x0F);
}


void lcdInit(void) {

	HAL_Delay(30);
	lcdSend4Bits(0x03);
	HAL_Delay(10);
	lcdSend4Bits(0x03);
	HAL_Delay(5);
	lcdSend4Bits(0x03);
	HAL_Delay(1);
	lcdSend4Bits(0x02);

	lcdSendCmnd(0x28);
	lcdSendCmnd(0x0C);
	lcdSendCmnd(0x06);
	lcdSendCmnd(0x01);
	lcdSendCmnd(0x0C); // 0x0C blink-off / 0x0F blink-on

	lcdDisplayMeasurement();
	lcdDisplaySetpoint();
	lcdDisplayMode();
}

void lcdSendData(uint8_t data) {

	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
    lcdSend4Bits(data >> 4);
    lcdSend4Bits(data & 0x0F);
}

void lcdSendString(char *str) {
	while (*str) {
		lcdSendData(*str++);
	}
}

void lcdDisplayMeasurement(void) {

	char string[10];
	sprintf(string, "%06.3f", getBatteryVoltage());
	lcdSendCmnd(0x80);
	lcdSendString(string);
	lcdSendData('V');
	sprintf(string, "%06.3f", getCurrent());
	lcdSendCmnd(0x89);
	lcdSendString(string);
	lcdSendData('A');
}

void lcdDisplaySetpoint(void) {

	char string[10];
	sprintf(string, "%06.3f", getSetpoint());
	lcdSendCmnd(0xC0);
	lcdSendString(string);

	if( getMode() == 'C') {
		lcdSendData('A'); lcdSendData(' ');
	}
	else if ( getMode() == 'R') {
		lcdSendData('O'); lcdSendData(' ');
	}
	else {
		lcdSendData('W'); lcdSendData(' ');
	}

}

void lcdDisplayMode(void) {

	lcdSendCmnd(0xC9);
	lcdSendData('C');
	lcdSendData(getMode());

	if (getOnOff() == 0) {

		lcdSendCmnd(0xCD);
		lcdSendString("OFF");
	}
	else {

		lcdSendCmnd(0xCD);
		lcdSendString(" ON");
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	volatile static uint16_t multiplyer = 1;
	static uint32_t last_clk_change = 0;
	static uint32_t last_sw_change = 0;
	uint32_t current_time = HAL_GetTick();

	// FUNCIONALITY OF BUTTON B1
	if ((GPIO_Pin == GPIO_PIN_13) && (current_time - last_sw_change > 500)) {

		switch(_display) {

			case DEFAULT:
				_display = TURN;
				break;
			case TURN:
				_display = MODE;
				break;
			case MODE:
				_display = LOGGING;
				break;
			case LOGGING:
				_display = DEFAULT;
				break;
			default:
				_display = DEFAULT;
				break;
		}
		lcd_disp_flag = ON;
		last_sw_change = current_time;
	}

	// FUNCTIONALITY OF CLK IMPULSATOR
	if ((GPIO_Pin == GPIO_PIN_12) && (current_time - last_clk_change > 200)) {

		switch(_display) {

			case DEFAULT:
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) != GPIO_PIN_RESET) {

					float new_setpoint = getSetpoint() + (0.001 * (float)multiplyer);
					setSetpoint(new_setpoint);
				}
				else {

					float new_setpoint = getSetpoint() - (0.001 * (float)multiplyer);
					setSetpoint(new_setpoint);
				}
				break;
			case TURN:
				lcd_menu_idx = (lcd_menu_idx >= ON) ? OFF : ON;
				break;
			case MODE:
				lcd_menu_idx += 1;
				if (lcd_menu_idx > 2)
					lcd_menu_idx = OFF;
				break;
			case LOGGING:
				lcd_menu_idx = (lcd_menu_idx >= ON) ? OFF : ON;
				break;
			default:
				break;
		}
		lcd_update_flag = ON;
		last_clk_change = current_time;
	}

	// FUNCTIONALITY OF SWITCH IMPULSATOR
	if ((GPIO_Pin == iSW_exti_Pin) && (current_time - last_sw_change > 250)) {

		switch(_display) {

			case DEFAULT:
				multiplyer *= 10;
				if (multiplyer >= 10000)
					multiplyer = 1;
				break;
			case TURN:
				setOnOff((lcd_menu_idx != 0) ? 1 : 0);
				break;
			case MODE:
				if (lcd_menu_idx == CC)
					setMode('C');
				else if (lcd_menu_idx == CR)
					setMode('R');
				else
					setMode('P');
				break;
			case LOGGING:
				setLogging((lcd_menu_idx != 0) ? 1 : 0);
				break;
			default:
				break;
		}
		lcd_update_flag = ON;
		last_sw_change = current_time;
	}
}

void lcdCheckForUpdate(void) {

	static float prev_setpoint = 0;
	static float prev_current = 0;
	static float prev_battery_voltage = 0;
	static uint8_t prev_mode = 'C';
	static uint8_t prev_onOff = 0;
	static uint8_t prev_logging = 0;
	static uint32_t last_change = 0;

	uint32_t current_time = HAL_GetTick();

	if (lcd_disp_flag == ON) {
		lcd_disp_flag = OFF;

		switch(_display) {

		case DEFAULT:
			lcdSendCmnd(0x01);
			lcdDisplayMeasurement();
			lcdDisplaySetpoint();
			lcdDisplayMode();
			last_change = current_time;
			break;
		case TURN:
			lcdSendCmnd(0x01);
			lcdSendCmnd(0x86);
			lcdSendString("[OFF]");
			lcdSendCmnd(0xC7);
			lcdSendString("ON");
			lcdSendCmnd(0xCE);
			lcdSendString("O/");
			if (getOnOff() == 0) {lcdSendCmnd(0x85); lcdSendData('_'); lcdSendCmnd(0xC5); lcdSendData(' ');}
			else {lcdSendCmnd(0xC5); lcdSendData('_'); lcdSendCmnd(0x85); lcdSendData(' ');}
			break;
		case MODE:
			// ADD a title
			lcdSendCmnd(0x01);
			lcdSendCmnd(0x81);
			lcdSendString("[CC]");
			lcdSendCmnd(0x8B);
			lcdSendString("CR");
			lcdSendCmnd(0xC7);
			lcdSendString("CP");
			lcdSendCmnd(0xCF);
			lcdSendString("M");
			if (getMode() == 'C') {lcdSendCmnd(0x80); lcdSendData('_'); lcdSendCmnd(0x89); lcdSendData(' '); lcdSendCmnd(0xC5); lcdSendData(' ');}
			else if (getMode() == 'R') {lcdSendCmnd(0x80); lcdSendData(' '); lcdSendCmnd(0x89); lcdSendData('_'); lcdSendCmnd(0xC5); lcdSendData(' ');}
			else {lcdSendCmnd(0x80); lcdSendData(' '); lcdSendCmnd(0x89); lcdSendData(' '); lcdSendCmnd(0xC5); lcdSendData('_');}
			break;
		case LOGGING:
			// ADD a title
			lcdSendCmnd(0x01);
			lcdSendCmnd(0x86);
			lcdSendString("[OFF]");
			lcdSendCmnd(0xC7);
			lcdSendString("ON");
			lcdSendCmnd(0xCF);
			lcdSendString("L");
			if (getLogging() == 0) {lcdSendCmnd(0x85); lcdSendData('_'); lcdSendCmnd(0xC5); lcdSendData(' ');}
			else {lcdSendCmnd(0xC5); lcdSendData('_'); lcdSendCmnd(0x85); lcdSendData(' ');}
			break;
		}
	}

	if (lcd_update_flag == ON) {
		lcd_update_flag = OFF;

		switch(_display) {

		case DEFAULT:
			break;
		case TURN:
			if (lcd_menu_idx == OFF) { lcdSendCmnd(0xC6); lcdSendData(' '); lcdSendCmnd(0xC9); lcdSendData(' ');
										lcdSendCmnd(0x86); lcdSendData('['); lcdSendCmnd(0x8A); lcdSendData(']');}
			else { lcdSendCmnd(0xC6); lcdSendData('['); lcdSendCmnd(0xC9); lcdSendData(']');
					lcdSendCmnd(0x86); lcdSendData(' '); lcdSendCmnd(0x8A); lcdSendData(' ');}
			break;
		case MODE:
			if (lcd_menu_idx == OFF) { lcdSendCmnd(0xC6); lcdSendData(' '); lcdSendCmnd(0xC9); lcdSendData(' ');
										lcdSendCmnd(0x8A); lcdSendData(' '); lcdSendCmnd(0x8D); lcdSendData(' ');
										lcdSendCmnd(0x81); lcdSendData('['); lcdSendCmnd(0x84); lcdSendData(']');}
			else if (lcd_menu_idx == ON) { lcdSendCmnd(0xC6); lcdSendData(' '); lcdSendCmnd(0xC9); lcdSendData(' ');
											lcdSendCmnd(0x8A); lcdSendData('['); lcdSendCmnd(0x8D); lcdSendData(']');
											lcdSendCmnd(0x81); lcdSendData(' '); lcdSendCmnd(0x84); lcdSendData(' ');}
			else { lcdSendCmnd(0xC6); lcdSendData('['); lcdSendCmnd(0xC9); lcdSendData(']');
					lcdSendCmnd(0x8A); lcdSendData(' '); lcdSendCmnd(0x8D); lcdSendData(' ');
					lcdSendCmnd(0x81); lcdSendData(' '); lcdSendCmnd(0x84); lcdSendData(' ');}
			break;
		case LOGGING:
			if (lcd_menu_idx == OFF) { lcdSendCmnd(0xC6); lcdSendData(' '); lcdSendCmnd(0xC9); lcdSendData(' ');
										lcdSendCmnd(0x86); lcdSendData('['); lcdSendCmnd(0x8A); lcdSendData(']');}
			else { lcdSendCmnd(0xC6); lcdSendData('['); lcdSendCmnd(0xC9); lcdSendData(']');
					lcdSendCmnd(0x86); lcdSendData(' '); lcdSendCmnd(0x8A); lcdSendData(' ');}
			break;
		}
	}

	if (_display == DEFAULT && (current_time-last_change) >= 3000) {		//____________________________

		if (prev_setpoint != getSetpoint()) {

			prev_setpoint = getSetpoint();
			lcdDisplaySetpoint();
		}

		if (prev_mode != getMode() || prev_onOff != getOnOff()) {

			prev_mode = getMode();
			prev_onOff = getOnOff();
			lcdDisplayMode();
		}

		if (prev_current != getCurrent() || prev_battery_voltage != getBatteryVoltage()) {

			prev_current = getCurrent();
			prev_battery_voltage = getBatteryVoltage();
			lcdDisplayMeasurement();
		}

		last_change = current_time;
	}

	if (_display == TURN || _display == LOGGING) {

		if ((prev_onOff != getOnOff() && getOnOff() == OFF) || (prev_logging != getLogging() && getLogging() == OFF)) {

			lcdSendCmnd(0x85); lcdSendData('_');
			lcdSendCmnd(0xC5); lcdSendData(' ');
			prev_onOff = getOnOff();
			prev_logging = getLogging();
		}
		else if ((prev_onOff != getOnOff() && getOnOff() == ON) || (prev_logging != getLogging() && getLogging() == ON)) {

			lcdSendCmnd(0xC5); lcdSendData('_');
			lcdSendCmnd(0x85); lcdSendData(' ');
			prev_onOff = getOnOff();
			prev_logging = getLogging();
		}
	}

	if (_display == MODE) {

		if (prev_mode != getMode() && getMode() == 'C') {

			lcdSendCmnd(0x80); lcdSendData('_');
			lcdSendCmnd(0x89); lcdSendData(' ');
			lcdSendCmnd(0xC5); lcdSendData(' ');
			prev_mode = getMode();
		}
		else if (prev_mode != getMode() && getMode() == 'R') {

			lcdSendCmnd(0x80); lcdSendData(' ');
			lcdSendCmnd(0x89); lcdSendData('_');
			lcdSendCmnd(0xC5); lcdSendData(' ');
			prev_mode = getMode();
		}
		else if (prev_mode != getMode() && getMode() == 'P') {

			lcdSendCmnd(0x80); lcdSendData(' ');
			lcdSendCmnd(0x89); lcdSendData(' ');
			lcdSendCmnd(0xC5); lcdSendData('_');
			prev_mode = getMode();
		}
	}
}
