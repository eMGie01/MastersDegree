/*
 * sd.h
 *
 *  Created on: Mar 17, 2025
 *      Author: marek
 */

#ifndef SRC_SD_H_
#define SRC_SD_H_

#include "rtc.h"
#include <stdint.h>

extern RTC_DateTypeDef sDate;
extern RTC_TimeTypeDef sTime;

extern uint8_t prev_logging;

extern uint16_t dummy_data[4];

void sd_attach(void);
void sd_dettach(void);
void getDateTime(char *date_time_str);
void createFolderWithCSV(char *path);
void addDataToBuffer(void);
void writeBufferToCsv(char buffer[][256], uint16_t size);

void test_sd_spi_basic(void);

#endif /* SRC_SD_H_ */
