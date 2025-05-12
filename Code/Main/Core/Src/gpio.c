/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC0   ------> I2C3_SCL
     PC1   ------> I2C3_SDA
     PA0   ------> UART4_TX
     PA1   ------> UART4_RX
     PC4   ------> USART3_TX
     PC5   ------> USART3_RX
     PB11   ------> I2C2_SDA
     PB13   ------> SPI2_SCK
     PB14   ------> SPI2_MISO
     PA9   ------> USART1_TX
     PA10   ------> USART1_RX
     PB6   ------> I2C1_SCL
     PB7   ------> I2C1_SDA
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_MAX30102_GPIO_Port, LED_MAX30102_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_SWD_Pin|LED_ERROR_Pin|LED_STATUS_Pin|LED_BLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GPS_Pin|LED_LCD_Pin|LED_BMP280_Pin|LED_MPUS6500_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ST7789_RST_GPIO_Port, ST7789_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ST7789_DC_Pin|ST7789_CS_Pin|ST7789_BLK_Pin|BLE_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MAX30102_SCL_Pin MAX30102_SDA_Pin */
  GPIO_InitStruct.Pin = MAX30102_SCL_Pin|MAX30102_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MAX30102_INT_Pin BUTTON_NEXT_Pin */
  GPIO_InitStruct.Pin = MAX30102_INT_Pin|BUTTON_NEXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_MAX30102_Pin ST7789_DC_Pin ST7789_CS_Pin ST7789_BLK_Pin
                           BLE_EN_Pin */
  GPIO_InitStruct.Pin = LED_MAX30102_Pin|ST7789_DC_Pin|ST7789_CS_Pin|ST7789_BLK_Pin
                          |BLE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_TX_Pin STLINK_RX_Pin */
  GPIO_InitStruct.Pin = STLINK_TX_Pin|STLINK_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_SWD_Pin LED_ERROR_Pin LED_STATUS_Pin LED_BLE_Pin */
  GPIO_InitStruct.Pin = LED_SWD_Pin|LED_ERROR_Pin|LED_STATUS_Pin|LED_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_TX_Pin GPS_RX_Pin */
  GPIO_InitStruct.Pin = GPS_TX_Pin|GPS_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GPS_Pin LED_LCD_Pin LED_BMP280_Pin ST7789_RST_Pin
                           LED_MPUS6500_Pin */
  GPIO_InitStruct.Pin = LED_GPS_Pin|LED_LCD_Pin|LED_BMP280_Pin|ST7789_RST_Pin
                          |LED_MPUS6500_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP280_SDA_Pin */
  GPIO_InitStruct.Pin = BMP280_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(BMP280_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7789_SCK_Pin ST7789_MISO_Pin */
  GPIO_InitStruct.Pin = ST7789_SCK_Pin|ST7789_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_BACK_Pin */
  GPIO_InitStruct.Pin = BUTTON_BACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_BACK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_TX_Pin BLE_RX_Pin */
  GPIO_InitStruct.Pin = BLE_TX_Pin|BLE_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_STATE_Pin */
  GPIO_InitStruct.Pin = BLE_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_STATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU6500_INT_Pin */
  GPIO_InitStruct.Pin = MPU6500_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU6500_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MPU6500_SCL_Pin MPU6500_SDA_Pin */
  GPIO_InitStruct.Pin = MPU6500_SCL_Pin|MPU6500_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
