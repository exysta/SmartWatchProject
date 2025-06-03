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
#include "stm32h7xx_hal.h"

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
#define InputButton_Pin GPIO_PIN_13
#define InputButton_GPIO_Port GPIOC
#define InputButton_EXTI_IRQn EXTI15_10_IRQn
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define PH0_MCU_Pin GPIO_PIN_0
#define PH0_MCU_GPIO_Port GPIOH
#define PH1_MCU_Pin GPIO_PIN_1
#define PH1_MCU_GPIO_Port GPIOH
#define MPU6500_AD0_Pin GPIO_PIN_3
#define MPU6500_AD0_GPIO_Port GPIOA
#define ST7789_SCK_Pin GPIO_PIN_5
#define ST7789_SCK_GPIO_Port GPIOA
#define ST7789_MOSI_Pin GPIO_PIN_7
#define ST7789_MOSI_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define MAX30102_INT_Pin GPIO_PIN_8
#define MAX30102_INT_GPIO_Port GPIOE
#define MAX30102_INT_EXTI_IRQn EXTI9_5_IRQn
#define BME280_CS_Pin GPIO_PIN_15
#define BME280_CS_GPIO_Port GPIOE
#define BME280_SCL_Pin GPIO_PIN_10
#define BME280_SCL_GPIO_Port GPIOB
#define BME280_SDA_Pin GPIO_PIN_11
#define BME280_SDA_GPIO_Port GPIOB
#define GNSS_RX_Pin GPIO_PIN_12
#define GNSS_RX_GPIO_Port GPIOB
#define GNSS_TX_Pin GPIO_PIN_13
#define GNSS_TX_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define MPU6500_SCL_Pin GPIO_PIN_12
#define MPU6500_SCL_GPIO_Port GPIOD
#define MPU6500_SDA_Pin GPIO_PIN_13
#define MPU6500_SDA_GPIO_Port GPIOD
#define ST7789_DC_Pin GPIO_PIN_14
#define ST7789_DC_GPIO_Port GPIOD
#define ST7789_RST_Pin GPIO_PIN_15
#define ST7789_RST_GPIO_Port GPIOD
#define ST7789_CS_Pin GPIO_PIN_9
#define ST7789_CS_GPIO_Port GPIOG
#define ST7789_BLK_Pin GPIO_PIN_4
#define ST7789_BLK_GPIO_Port GPIOB
#define MAX30102_SCL_Pin GPIO_PIN_6
#define MAX30102_SCL_GPIO_Port GPIOB
#define MAX30102_SDA_Pin GPIO_PIN_7
#define MAX30102_SDA_GPIO_Port GPIOB
#define BLE_RX_Pin GPIO_PIN_8
#define BLE_RX_GPIO_Port GPIOB
#define BLE_TX_Pin GPIO_PIN_9
#define BLE_TX_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
