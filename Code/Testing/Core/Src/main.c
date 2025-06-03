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
#include "dma.h"
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "GNSS.h"
#include "stm32h7xx_hal.h"
#include <string.h>         // For memset(), strlen(), sprintf()
#include <stdio.h>          // For printf() (if using debugging via UART)
#include <st7789.h>
#include "MPU6500_driver.h"
#include "ble_comms.h"
#include "common_defs.h"
#include "display.h"
#include "max30102_for_stm32_hal.h"
#include "sensors.h"
#include "uart_comms.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float pressure, temperature, humidity;

uint16_t size;
uint8_t Data[256];

GNSS_StateHandle GNSS_Handle;

volatile uint16_t rxPos = 0;
char messageBuffer[RX_BUFFER_SIZE];
volatile uint8_t messageReady = 0;
extern uint8_t rxBuffer[RX_BUFFER_SIZE];

extern SmartWatchData_t SmartWatchData_handle;
UI_Screen_State_t SmartWatchScreen_State;
max30102_t max30102;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int chr)
{
	HAL_UART_Transmit(&STLINK_UART, (uint8_t*) &chr, 1, HAL_MAX_DELAY);
	return chr;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  MX_SPI1_Init();
  MX_I2C4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	HAL_StatusTypeDef Scan_I2C_Bus(I2C_HandleTypeDef *hi2c)
	{
		HAL_StatusTypeDef status;
		uint32_t err;
		for (uint16_t addr = 1; addr < 128; addr++)
		{
			status = HAL_I2C_IsDeviceReady(hi2c, addr << 1, 3, 500);
			if (status == HAL_OK)
			{
				printf("I2C: device ACK at 0x%02X\r\n", addr);
			}
			else
			{
				err = HAL_I2C_GetError(hi2c);
				// err == HAL_I2C_ERROR_NONE usually means NACK
				printf("I2C: 0x%02X no ACK (err=0x%lX)\r\n", addr, err);
			}
			HAL_Delay(5);  // give time for UART to flush
		}
		return HAL_OK;
	}

#ifdef BLE_TEST

  configureHM10();
  startUartReception(&BLE_UART);
#endif

#ifdef BME280
	Scan_I2C_Bus(&BME280_I2C);

  Sensor_BMP280_init(&SmartWatchData_handle.bmp280);
#endif

#ifdef GPS_TEST
//	GNSS_Init(&GNSS_Handle, &GNSS_UART);
//	HAL_Delay(1000);
//	GNSS_LoadConfig(&GNSS_Handle);
//	uint32_t Timer = HAL_GetTick();
    Sensor_GNSS_Init(&SmartWatchData_handle,&GNSS_UART);        // init GNSS data handle

#endif

#ifdef MPU6500_TEST

	/* WHO_AM_I read */
	uint8_t MPU6500_ReadWhoAmI(void)
	{
	    uint8_t who_am_i = 0;
	    HAL_I2C_Mem_Read(&MPU6500_I2C, MPU6500_I2C_ADDR_SHIFTED, MPU6500_RGSTR_WHO_AM_I,
	                     I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, HAL_MAX_DELAY);
	    return who_am_i;
	}

	/* Read values */



    uint8_t id = MPU6500_ReadWhoAmI();
    printf("mpu6500: WHO_AM_I = 0x%02X\r\n", id);

#endif

#ifdef SCREEN_TEST
	SmartWatchScreen_State = SCREEN_ENVIRONMENTAL;
	Display_Init(SmartWatchScreen_State);
	ST7789_Test();
#endif

#ifdef MAX30102_TEST
	// 7-bit I2C address of the MAX30102 is 0x57, shift left for HAL (8-bit format)
#define MAX30102_ADDR   (0x57 << 1)

	// MAX30102 register addresses
#define MAX30102_PART_ID_REG   0xFF

	// timeout for HAL transactions (ms)
#define MAX30102_I2C_TIMEOUT   100

	/**
	 * @brief  Simple connection test for MAX30102
	 * @retval HAL status: HAL_OK if device responded with correct PART_ID,
	 *         HAL_ERROR otherwise (or bus error)
	 */
	HAL_StatusTypeDef MAX30102_TestConnection(void)
	{
		uint8_t part_id = 0;
		HAL_StatusTypeDef status;

		// Read PART_ID register
		status = HAL_I2C_Mem_Read(&MAX30102_I2C,
		MAX30102_ADDR,
		MAX30102_PART_ID_REG,
		I2C_MEMADD_SIZE_8BIT, &part_id, 1,
		MAX30102_I2C_TIMEOUT);

		if (status != HAL_OK)
		{
			// I2C error (NACK, bus fault, etc.)
			return status;
		}

		if (part_id != 0x15)
		{
			// Unexpected ID
			return HAL_ERROR;
		}

		// All good!
		return HAL_OK;
	}

//	HAL_StatusTypeDef test = MAX30102_TestConnection();
	Scan_I2C_Bus();
//	Sensor_MAX30102_init(800, &max30102, &MAX30102_I2C);
	Sensor_MAX30102_init(&SmartWatchData_handle.max30102, &MAX30102_I2C);

#endif
//	Scan_I2C_Bus(&MAX30102_I2C);
	uint32_t Timer = HAL_GetTick();
	Sensor_SmartWatch_init(&SmartWatchData_handle);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
#ifdef BME280

		Sensor_BMP280_read_data(&SmartWatchData_handle);


		printf("Pressure: %.2f Pa, Temperature: %.2f C,  Humidity: %.2f \r\n", SmartWatchData_handle.pressure, SmartWatchData_handle.temperature,SmartWatchData_handle.humidity);


		HAL_Delay(500);
#endif

#ifdef GPS_TEST
//		if ((HAL_GetTick() - Timer) > 1000)
//		{
//			GNSS_GetUniqID(&GNSS_Handle);
//			GNSS_ParseBuffer(&GNSS_Handle);
//			HAL_Delay(250);
//			GNSS_GetPVTData(&GNSS_Handle);
//			GNSS_ParseBuffer(&GNSS_Handle);
//			HAL_Delay(250);
//			GNSS_SetMode(&GNSS_Handle,Wirst);
//			HAL_Delay(250);
//			printf("Day: %d-%d-%d \r\n", GNSS_Handle.day, GNSS_Handle.month,GNSS_Handle.year);
//			printf("Time: %d:%d:%d \r\n", GNSS_Handle.hour, GNSS_Handle.min,GNSS_Handle.sec);
//			printf("Status of fix: %d \r\n", GNSS_Handle.fixType);
//			printf("Latitude: %f \r\n", GNSS_Handle.fLat);
//			printf("Longitude: %f \r\n",(float) GNSS_Handle.lon / 10000000.0);
//			printf("Height above ellipsoid: %d \r\n", GNSS_Handle.height);
//			printf("Height above mean sea level: %d \r\n", GNSS_Handle.hMSL);
//			printf("Ground Speed (2-D): %d \r\n", GNSS_Handle.gSpeed);
//			printf("Unique ID: %04X %04X %04X %04X %04X \n\r",
//					GNSS_Handle.uniqueID[0], GNSS_Handle.uniqueID[1],
//					GNSS_Handle.uniqueID[2], GNSS_Handle.uniqueID[3],
//					GNSS_Handle.uniqueID[4], GNSS_Handle.uniqueID[5]);
//			printf("--------------------------------------\r\n" );
//			Timer = HAL_GetTick();
//		}
		Sensor_GNSS_Update(&SmartWatchData_handle);

		Sensor_SmartWatch_log(&SmartWatchData_handle);
		//			Timer = HAL_GetTick();

		HAL_Delay(1000);
#endif
#ifdef BLE_TEST
        if (messageReady) {
            // Process the complete message
            printf("Received complete message: %s\r\n", messageBuffer);
            messageReady = 0;
        }
#endif

#ifdef MPU6500_TEST
        Sensor_MPU6500_read_data(&SmartWatchData_handle);
        printf("===================================\r\n");

		HAL_Delay(500);

#endif

#ifdef SCREEN_TEST

		SmartWatchData_handle.pressure += 1;
		SmartWatchData_handle.heart_rate += 1;
		SmartWatchData_handle.spo2 += 1;
		SmartWatchScreen_State = SCREEN_HEART_RATE;

	    Display_Update(SmartWatchScreen_State, &SmartWatchData_handle);
//	    HAL_Delay(20);
//		Display_EnvironnementData(30,70,&SmartWatchData_handle);
#endif

#ifdef MAX30102_TEST
		// If interrupt flag is active
		Sensor_max30102_Update(&SmartWatchData_handle);
//	    if (max30102_has_interrupt(&SmartWatchData_handle.max30102))
//	    {
//		      // Run interrupt handler to read FIFO
//		      max30102_interrupt_handler(&SmartWatchData_handle.max30102);
//		      Sensor_MAX30102_compute(&SmartWatchData_handle.max30102);
//		      uint8_t hr = Sensor_MAX30102_get_hr();
//		      uint8_t spo2 = Sensor_MAX30102_get_spo2();
//
//		      printf("---------------new data---------------\r\n");
//		      printf("hr : %u r\n",hr);
//		      printf("spo2 : %u r\n",spo2);
//
//	    }

#endif
		if (messageReady)
		{
			// Process the complete message
			printf("Received complete message: %s\r\n", messageBuffer);
			messageReady = 0;
		}

		Sensor_SmartWatch_update(&SmartWatchData_handle);

		if ((HAL_GetTick() - Timer) > 1000)
		{
			Sensor_SmartWatch_log(&SmartWatchData_handle);
			Timer = HAL_GetTick();
		}
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

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// This callback is called when idle line is detected or buffer is full
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == UART4)
	{
		// Copy the data from DMA buffer to message buffer
		memcpy(messageBuffer, rxBuffer, Size);

		// Null-terminate the string
		messageBuffer[Size] = '\0';

		// Set flag for main loop
		messageReady = 1;

		HAL_UART_AbortReceive(huart);  // Stop DMA
		memset(rxBuffer, 0, sizeof(rxBuffer));  // Reset buffer

		// Restart DMA reception
		HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, RX_BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == InputButton_Pin)
	{

	}
	else if (GPIO_Pin == MAX30102_INT_Pin)
	{
		max30102_on_interrupt(&SmartWatchData_handle.max30102);
	}
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
