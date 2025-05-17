/*
 * uart_comms.h
 *
 *  Created on: May 14, 2025
 *      Author: exysta
 */

#ifndef SMARTWATCHDRIVERS_INCLUDE_UART_COMMS_H_
#define SMARTWATCHDRIVERS_INCLUDE_UART_COMMS_H_

void SendSmartWatchData(UART_HandleTypeDef *huart, const SmartWatchData_t *sw);
void SendScreenState(UART_HandleTypeDef *huart, UI_Screen_State_t state);
void Sensor_SmartWatch_log(const SmartWatchData_t *sw);


#endif /* SMARTWATCHDRIVERS_INCLUDE_UART_COMMS_H_ */
