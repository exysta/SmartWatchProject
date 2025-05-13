/*
 * ble_comms.h
 *
 *  Created on: May 5, 2025
 *      Author: exysta
 */

#ifndef INC_BLE_COMMS_H_
#define INC_BLE_COMMS_H_

#include "usart.h"


void sendATCommand(UART_HandleTypeDef *huart, char *command);
void configureHM10();
void startUartReception(UART_HandleTypeDef *huart);
HAL_StatusTypeDef BLE_Send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len, uint32_t timeout);


#endif /* INC_BLE_COMMS_H_ */
