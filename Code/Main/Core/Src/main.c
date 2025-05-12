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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"
#include "bmp280.h"
#include "GNSS.h"
#include <string.h>         // For memset(), strlen(), sprintf()
#include <stdio.h>          // For printf() (if using debugging via UART)
#include <st7789.h>
#include "MPU6500_driver.h"
#include "ble_comms.h"
#include "common_defs.h"
#include "display.h"
#include "max30102_for_stm32_hal.h"
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int chr)
{
	HAL_UART_Transmit(&STLINK_UART, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

#ifdef BLE_TEST

  configureHM10();
  startUartReception(&BLE_UART);
#endif

#ifdef BME280
	uint8_t txData = 0xD0;  // Array with a single element
	// Command to send
	uint8_t rxData[1];           // Buffer to store response

	HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, &txData, 1, 100);
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&BME280_I2C, BME280_ADDR, rxData, 1, 100);
	printf("Received: 0x%02X \r\n", rxData[0]);  // Print response


	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &BME280_I2C;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		size = sprintf((char *)Data, "BMP280 initialization failed\r\n");
		HAL_UART_Transmit(&huart3, Data, size, 1000);
		HAL_Delay(2000);
	}
	bool bme280p = bmp280.id == BME280_CHIP_ID;
	size = sprintf((char *)Data, "BMP280: found %s \r\n", bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart3, Data, size, 1000);
#endif

#ifdef GPS_TEST
	GNSS_Init(&GNSS_Handle, &GNSS_UART);
	HAL_Delay(1000);
	GNSS_LoadConfig(&GNSS_Handle);
	uint32_t Timer = HAL_GetTick();
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

#endif


#ifdef MAX30102_I2C
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
	                              I2C_MEMADD_SIZE_8BIT,
	                              &part_id,
	                              1,
	                              MAX30102_I2C_TIMEOUT);

	    if (status != HAL_OK) {
	        // I2C error (NACK, bus fault, etc.)
	        return status;
	    }

	    if (part_id != 0x15) {
	        // Unexpected ID
	        return HAL_ERROR;
	    }

	    // All good!
	    return HAL_OK;
	}
	HAL_StatusTypeDef Scan_I2C_Bus(void)
	{
	    HAL_StatusTypeDef status;
	    uint32_t err;
	    for (uint16_t addr = 1; addr < 128; addr++)
	    {
	        status = HAL_I2C_IsDeviceReady(&MAX30102_I2C,
	                                       addr << 1,
	                                       3,
	                                       MAX30102_I2C_TIMEOUT);
	        if (status == HAL_OK)
	        {
	            printf("I2C: device ACK at 0x%02X\r\n", addr);
	        }
	        else
	        {
	            err = HAL_I2C_GetError(&MAX30102_I2C);
	            // err == HAL_I2C_ERROR_NONE usually means NACK
	            printf("I2C: 0x%02X no ACK (err=0x%lX)\r\n", addr, err);
	        }
	        HAL_Delay(5);  // give time for UART to flush
	    }
	    return HAL_OK;
	}
//	HAL_StatusTypeDef test = MAX30102_TestConnection();
	  Scan_I2C_Bus();
	  // Initiation
	  max30102_init(&max30102, &MAX30102_I2C);
	  max30102_reset(&max30102);
	  max30102_clear_fifo(&max30102);
	  max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);

	  // Sensor settings
	  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
	  max30102_set_adc_resolution(&max30102, max30102_adc_2048);
	  max30102_set_sampling_rate(&max30102, max30102_sr_800);
	  max30102_set_led_current_1(&max30102, 6.2);
	  max30102_set_led_current_2(&max30102, 6.2);

	  // Enter SpO2 mode
	  max30102_set_mode(&max30102, max30102_spo2);
	  max30102_set_a_full(&max30102, 1);

	  // Initiate 1 temperature measurement
	  max30102_set_die_temp_en(&max30102, 1);
	  max30102_set_die_temp_rdy(&max30102, 1);

	  uint8_t en_reg[2] = {0};
	  max30102_read(&max30102, 0x00, en_reg, 1);

	//Enter measurement mode:
	// Enter SpO2 mode
	max30102_set_mode(&max30102, max30102_spo2);

	//Enable the required interrupts:
	// Enable FIFO_A_FULL interrupt
	max30102_set_a_full(&max30102, 1);
	// Enable die temperature measurement
	max30102_set_die_temp_en(&max30102, 1);
	// Enable DIE_TEMP_RDY interrupt
	max30102_set_die_temp_rdy(&max30102, 1);
	printf("test\r\n");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef BME280

		HAL_Delay(100);

		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			printf("Temperature/pressure reading failed\r\n");
			HAL_Delay(2000);
		}

		printf("Pressure: %.2f Pa, Temperature: %.2f C \r\n", pressure, temperature);

		if (bme280p) {
			printf(", Humidity: %.2f\r\n", humidity);
		} else {
			printf("\r\n");
		}

		HAL_Delay(2000);
#endif


#ifdef GPS_TEST
		if ((HAL_GetTick() - Timer) > 1000) {
			GNSS_GetUniqID(&GNSS_Handle);
			GNSS_ParseBuffer(&GNSS_Handle);
			HAL_Delay(250);
			GNSS_GetPVTData(&GNSS_Handle);
			GNSS_ParseBuffer(&GNSS_Handle);
			HAL_Delay(250);
			GNSS_SetMode(&GNSS_Handle,Automotiv);
			HAL_Delay(250);
			printf("Day: %d-%d-%d \r\n", GNSS_Handle.day, GNSS_Handle.month,GNSS_Handle.year);
			printf("Time: %d:%d:%d \r\n", GNSS_Handle.hour, GNSS_Handle.min,GNSS_Handle.sec);
			printf("Status of fix: %d \r\n", GNSS_Handle.fixType);
			printf("Latitude: %f \r\n", GNSS_Handle.fLat);
			printf("Longitude: %f \r\n",(float) GNSS_Handle.lon / 10000000.0);
			printf("Height above ellipsoid: %d \r\n", GNSS_Handle.height);
			printf("Height above mean sea level: %d \r\n", GNSS_Handle.hMSL);
			printf("Ground Speed (2-D): %d \r\n", GNSS_Handle.gSpeed);
			printf("Unique ID: %04X %04X %04X %04X %04X \n\r",
					GNSS_Handle.uniqueID[0], GNSS_Handle.uniqueID[1],
					GNSS_Handle.uniqueID[2], GNSS_Handle.uniqueID[3],
					GNSS_Handle.uniqueID[4], GNSS_Handle.uniqueID[5]);
			printf("--------------------------------------\r\n" );
			Timer = HAL_GetTick();
		}
#endif
#ifdef BLE_TEST
        if (messageReady) {
            // Process the complete message
            printf("Received complete message: %s\r\n", messageBuffer);
            messageReady = 0;
        }
#endif



#ifdef MPU6500_TEST
	    read_mpu_data_example();
	    printf("===================================\r\n");
		HAL_Delay(1000);

#endif

#ifdef SCREEN_TEST

//	    for (int i = 0; i < 100; ++i) {
//	    	SmartWatchData_handle.pressure = 100.0f + 10.0f * (i % 5); // 100, 110, 120, 130, 140, repeat
//	    	HAL_Delay(80);
//	    }
		SmartWatchData_handle.pressure += 1;
		SmartWatchData_handle.heart_rate += 1;
		SmartWatchData_handle.spo2 += 1;
		SmartWatchScreen_State = SCREEN_HEART_RATE;
	    Display_Update(SmartWatchScreen_State, &SmartWatchData_handle);
	    HAL_Delay(20);
		//Display_EnvironnementData(30,70,&SmartWatchData_handle);
#endif

#ifdef MAX30102_I2C
	    // If interrupt flag is active
	    if (max30102_has_interrupt(&max30102))
	      // Run interrupt handler to read FIFO
	      max30102_interrupt_handler(&max30102);
#endif
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// This callback is called when idle line is detected or buffer is full
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	//for BLE
    if (huart == &BLE_UART)
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
    if (GPIO_Pin == BUTTON_BACK_Pin)
    {

    }
    else if (GPIO_Pin == BUTTON_NEXT_Pin)
    {

    }
    else if (GPIO_Pin == MAX30102_INT_Pin)
    {
    	max30102_on_interrupt(&max30102);
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
