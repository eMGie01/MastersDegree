#include "sd.h"
#include "spi.h"
#include "usart.h"
#include "fatfs.h"
#include "mystruct.h"
#include "komunikacjaPC2.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#define MAX_SD_BUFFER_SIZE 64
#define DATE_TIME_LENGTH 16

FATFS fs;
FIL file;
FRESULT res;

uint8_t prev_logging = 0;

char file_path[48];

//void sd_attach(void) {
//
//	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
//}
//void sd_dettach(void) {
//
//	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//}

#define SD_CS_LOW()   HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

// Podłączenie SD (inicjalizacja)
void sd_attach(void) {
    SD_CS_LOW();
}

// Odłączenie SD
void sd_dettach(void) {
    SD_CS_HIGH();
}

void getDateTime(char *date_time_str) {

	RTC_DateTypeDef sDate;
	RTC_TimeTypeDef sTime;

    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    snprintf(date_time_str, DATE_TIME_LENGTH, "%02d%02d%02d%02d", sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes);
}

void createFolderWithCSV(char *path) {

	char file_name[22];
	char file_path[48];

	sd_attach();
	HAL_Delay(20);

	res = f_mount(&fs, "", 1);
	if (res != FR_OK) {
		myprintf("f_mount error (%i)\r\n", res);
		prev_logging = 0;
		setLogging(0);
		return;
	} else {
		myprintf("SD card mount successfully\r\n");
	}

	getDateTime(&file_name[0]);

	res = f_mkdir(file_name);
	if (res == FR_OK) {
		myprintf("Folder '%s' was created\r\n", file_name);
	} else if (res == FR_EXIST) {
		myprintf("Folder '%s' already exists\r\n", file_name);
	} else {
		myprintf("Error with creating folder %s, (%i)\r\n", file_name, res);
		f_mount(NULL, "", 0);
		sd_dettach();
		return;
	}

	snprintf(file_path, sizeof(file_path), "%s/%s.csv", file_name, file_name);

	memcpy(path, file_path, strlen(file_path) + 1);

	res = f_open(&file, file_path, FA_WRITE | FA_CREATE_ALWAYS);
	if (res != FR_OK) {
		myprintf("Error opening .csv file (%i)\r\n", res);
	} else {
		char header[] = "time, setpoint, parameter, current, battery voltage, DAC, temp_1, temp_2, ON/OFF, qlmb\r\n";
		UINT bytes_written;
		res = f_write(&file, header, strlen(header), &bytes_written);
		if (res != FR_OK) myprintf("Writing header ERROR (%i)\r\n", res);

		f_close(&file);
	}

	f_mount(NULL, "", 0);
	sd_dettach();
}

void addDataToBuffer(void) {
    static uint32_t time_base = 0;
    static float time = 0.0f;

    static char sd_buffer[MAX_SD_BUFFER_SIZE][256];
    static uint16_t sd_buffer_idx = 0;

    if (prev_logging == 0 && getLogging() == 1) {
        prev_logging = 1;
        createFolderWithCSV(&file_path[0]);
        time_base = HAL_GetTick();
        time = 0.0f;
        sd_buffer_idx = 0;
    } else {
        time = (float)(HAL_GetTick() - time_base) / 1000.0f;
    }

    // Collect current timestamp and data
    RTC_DateTypeDef sDate;
    RTC_TimeTypeDef sTime;
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    // Format line (safe and \r\n newline)
    int len = snprintf(sd_buffer[sd_buffer_idx], sizeof(sd_buffer[sd_buffer_idx]),
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%d,%.2f\r\n",
        time, getSetpoint(), getParameter(), getCurrent(), getBatteryVoltage(),
        getDacOutput(), temp_1, temp_3, getOnOff(), qlmb);

    if (len < 0) {
        myprintf("Encoding error in snprintf!\r\n");
        return;
    }
    if (len >= sizeof(sd_buffer[sd_buffer_idx])) {
        myprintf("CSV line too long (%d), skipping row to avoid overflow!\r\n", len);
        return;
    }

    sd_buffer_idx++;

    if (sd_buffer_idx >= getLoggingBufferSize()) {
        writeBufferToCsv(sd_buffer, sd_buffer_idx);
        sd_buffer_idx = 0;
    }
}

void writeBufferToCsv(char buffer[][256], uint16_t size) {
    sd_attach();

    f_mount(&fs, "", 1);

    res = f_open(&file, file_path, FA_WRITE | FA_OPEN_APPEND);
    if (res == FR_OK) {
        UINT bytes_written;

        for (int i = 0; i < size; i++) {
            res = f_write(&file, buffer[i], strlen(buffer[i]), &bytes_written);
            if (res != FR_OK) {
                myprintf("f_write error (%i)\r\n", res);
                break;
            }
        }

        myprintf("Data was added to .csv\r\n");
        f_close(&file);

        for (int i = 0; i < size; i++) {
            memset(buffer[i], 0, 256);
        }

    } else {
        myprintf("ERROR CSV (%i)\r\n", res);
    }

    f_mount(NULL, "", 0);
    sd_dettach();
}

void test_sd_spi_basic(void) {
    uint8_t tx = 0xFF;
    uint8_t rx = 0x00;

    // CS = LOW (karta aktywna)
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

    HAL_Delay(1); // krótki czas na "settling"

    // Wyślij bajt 0xFF przez SPI
    HAL_SPI_TransmitReceive(&hspi3, &tx, &rx, 1, HAL_MAX_DELAY);

    // CS = HIGH (koniec transakcji)
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

    myprintf("Odebrany bajt z SD: 0x%02X\r\n", rx);
}
