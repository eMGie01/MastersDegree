/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "komunikacjaPC2.h"
#include "interface.h"
#include "regulator.h"
#include "mystruct.h"
#include "sd.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define V_MAX_DAC 	3.251
#define V_DAC 		0.000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t timer_flag = 0;
float batteryVoltage = 0.0;
float current = 0.0;
float dac_output = 0.0;
float setpoint = 0.0;
float temperature_1 = 0.0;
float temperature_3 = 0.0;
float parameter = 0.0;
volatile float time_1 = 0.0;
float timeStop1 = 0.0f;
uint8_t onOff1 = 0;
uint8_t logg = 0;

float kp = 0.0;
float ki = 0.0;
float kd = 0.0;

float wire_resist = 0.0f;

uint16_t buffer_length = 0;

uint16_t loggingSpeed;
uint16_t loggingBuffer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &uart_byte, 1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim6);
  createDummyData( &myData);
  lcdInit();
  MCP3561T_1_Init(CS_1_GPIO_Port, CS_1_Pin);
  MCP3561T_2_Init(CS_2_GPIO_Port, CS_2_Pin);
  HAL_GPIO_WritePin(WENT_GPIO_Port, WENT_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t sd_idx = 0;
//  uint16_t lcd_idx = 0;
  uint32_t temp_time = 0;
  uint8_t temp_flag = 0;
  float prev_temp= 0.0f;
//  uint32_t prev_time = HAL_GetTick();
  uint16_t speed = 0;
  uint32_t time_base = 0;
  uint8_t prevOnOff = 0;

  while (1)
  {
	  batteryVoltage = getBatteryVoltage();
	  current = getCurrent();
	  dac_output = getDacOutput();
	  setpoint = getSetpoint();
	  temperature_1 = getTemp_1();
	  temperature_3 = getTemp_3();
	  parameter = getParameter();
	  loggingSpeed = getLoggingSpeed();
	  loggingBuffer = getLoggingBufferSize();
	  kp = Kp;
	  ki = Ki;
	  kd = Kd;
	  timeStop1 = timeStop;
	  logg = getLogging();
	  onOff1 = getOnOff();
	  wire_resist = *(float*)&myData.data[43];

	  if (prevOnOff == 0 && getOnOff() == 1) {

		  prevOnOff = 1;
		  time_base = HAL_GetTick();
	  }

	  if (getOnOff() != 1) {
		  prevOnOff = 0;
		  qlmb = 0;
	  }

	  if (timer_flag) {
		  timer_flag = 0;

		  sd_idx += 1;
		  float minutesPassed = timeStop - (float)((HAL_GetTick() - time_base) / (1000.0*60.0));
		  if (minutesPassed < 0) {
			  setOnOff(0);
			  setLogging(0);
		  }
	  }

	  if (getBatteryVoltage() <= 11.0)
		  setOnOff(0);

	  if (qlmb/3600 > 250)
		  setOnOff(0);

	  if (getTemp_1() >= 170 && prev_temp >= 170) {

		  setDacOutput(0);
		  setOnOff(0);
	  }

	  if (temp_flag != 1) {

		  measureTemp_1();
		  measureTemp_3();
		  temp_time = HAL_GetTick();
		  temp_flag = 1;
	  }
	  else if (temp_flag && (HAL_GetTick() - temp_time >= 710)) {

		  temp_flag = 0;
		  setTemp_1();
		  setTemp_3();
	  }
	  prev_temp = getTemp_1();

	  if ((float)((HAL_GetTick() - time_base) / (1000.0*60.0)) < 1.0) {
		  speed = 100;
	  }
	  else {
		  speed = getLoggingSpeed();;
	  }
	  if (sd_idx >= speed/50 && getLogging() == 1) {

		  sd_idx = 0;
		  addDataToBuffer();
	  }


	  lcdCheckForUpdate();
	  checkForData( &myData);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM2) {

        timer_flag = 1;
        setCurrent();
        setBatteryVoltage();
		setParameter();
        regulatorPI();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
