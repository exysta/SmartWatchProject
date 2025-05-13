/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MAX30102_SCL_Pin GPIO_PIN_0
#define MAX30102_SCL_GPIO_Port GPIOC
#define MAX30102_SDA_Pin GPIO_PIN_1
#define MAX30102_SDA_GPIO_Port GPIOC
#define MAX30102_INT_Pin GPIO_PIN_2
#define MAX30102_INT_GPIO_Port GPIOC
#define MAX30102_INT_EXTI_IRQn EXTI2_IRQn
#define LED_MAX30102_Pin GPIO_PIN_3
#define LED_MAX30102_GPIO_Port GPIOC
#define STLINK_TX_Pin GPIO_PIN_0
#define STLINK_TX_GPIO_Port GPIOA
#define STLINK_RX_Pin GPIO_PIN_1
#define STLINK_RX_GPIO_Port GPIOA
#define LED_SWD_Pin GPIO_PIN_2
#define LED_SWD_GPIO_Port GPIOA
#define LED_ERROR_Pin GPIO_PIN_5
#define LED_ERROR_GPIO_Port GPIOA
#define LED_STATUS_Pin GPIO_PIN_6
#define LED_STATUS_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_4
#define GPS_TX_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_5
#define GPS_RX_GPIO_Port GPIOC
#define LED_GPS_Pin GPIO_PIN_0
#define LED_GPS_GPIO_Port GPIOB
#define LED_LCD_Pin GPIO_PIN_1
#define LED_LCD_GPIO_Port GPIOB
#define LED_BMP280_Pin GPIO_PIN_2
#define LED_BMP280_GPIO_Port GPIOB
#define BMP280_SCL_Pin GPIO_PIN_10
#define BMP280_SCL_GPIO_Port GPIOB
#define BMP280_SDA_Pin GPIO_PIN_11
#define BMP280_SDA_GPIO_Port GPIOB
#define ST7789_RST_Pin GPIO_PIN_12
#define ST7789_RST_GPIO_Port GPIOB
#define ST7789_SCK_Pin GPIO_PIN_13
#define ST7789_SCK_GPIO_Port GPIOB
#define ST7789_MOSI_Pin GPIO_PIN_15
#define ST7789_MOSI_GPIO_Port GPIOB
#define ST7789_DC_Pin GPIO_PIN_6
#define ST7789_DC_GPIO_Port GPIOC
#define ST7789_CS_Pin GPIO_PIN_7
#define ST7789_CS_GPIO_Port GPIOC
#define ST7789_BLK_Pin GPIO_PIN_8
#define ST7789_BLK_GPIO_Port GPIOC
#define BUTTON_NEXT_Pin GPIO_PIN_9
#define BUTTON_NEXT_GPIO_Port GPIOC
#define BUTTON_NEXT_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_BACK_Pin GPIO_PIN_8
#define BUTTON_BACK_GPIO_Port GPIOA
#define BUTTON_BACK_EXTI_IRQn EXTI9_5_IRQn
#define BLE_TX_Pin GPIO_PIN_9
#define BLE_TX_GPIO_Port GPIOA
#define BLE_RX_Pin GPIO_PIN_10
#define BLE_RX_GPIO_Port GPIOA
#define LED_BLE_Pin GPIO_PIN_11
#define LED_BLE_GPIO_Port GPIOA
#define BLE_EN_Pin GPIO_PIN_10
#define BLE_EN_GPIO_Port GPIOC
#define BLE_STATE_Pin GPIO_PIN_11
#define BLE_STATE_GPIO_Port GPIOC
#define MPU6500_INT_Pin GPIO_PIN_5
#define MPU6500_INT_GPIO_Port GPIOB
#define MPU6500_INT_EXTI_IRQn EXTI9_5_IRQn
#define MPU6500_SCL_Pin GPIO_PIN_6
#define MPU6500_SCL_GPIO_Port GPIOB
#define MPU6500_SDA_Pin GPIO_PIN_7
#define MPU6500_SDA_GPIO_Port GPIOB
#define LED_MPUS6500_Pin GPIO_PIN_8
#define LED_MPUS6500_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
