/*
 * uart_comms.c
 *
 *  Created on: May 14, 2025
 *      Author: exysta
 */

// this is to communicate with another stm32 board over uart.
#include "main.h"   // for UART_HandleTypeDef, HAL_UART_Transmit, etc.
#include "common_defs.h" // for SmartWatchData_t, UI_Screen_State_t, NUM_SCREENS
#include <string.h> // for SmartWatchData_t, UI_Screen_State_t, NUM_SCREENS
#include <stdio.h> // for SmartWatchData_t, UI_Screen_State_t, NUM_SCREENS


// Transmit all key fields of your SmartWatchData_t as a single CSV line.
// You can adjust which fields you include or change formatting as needed.
void SendSmartWatchData(UART_HandleTypeDef *huart, const SmartWatchData_t *sw)
{
    // Note: if you really need EVERY sample from accel_g[][] and gyro_dps[][],
    // you'll need to loop and send multiple lines or switch to a binary protocol.
    // Here we just send the most recent sample [0].
    char buf[256];
    int len = snprintf(buf, sizeof(buf),
        "T=%.2f, P=%.1f, H=%.2f, "
        "Ax=%.3f, Ay=%.3f, Az=%.3f, "
        "Gx=%.3f, Gy=%.3f, Gz=%.3f, "
        "HR=%d, SpO2=%.1f, "
        "Lat=%.6f, Lon=%.6f, Alt=%.2f, "
        "Date=%04u-%02u-%02u, Time=%02u:%02u:%02u, Fix=%u\r\n",
        sw->temperature,
        sw->pressure,
        sw->humidity,
        sw->accel_g[0][0],
        sw->accel_g[0][1],
        sw->accel_g[0][2],
        sw->gyro_dps[0][0],
        sw->gyro_dps[0][1],
        sw->gyro_dps[0][2],
        sw->heart_rate,
        sw->spo2,
        sw->latitude,
        sw->longitude,
        sw->altitude,
        sw->year,
        sw->month,
        sw->day,
        sw->hour,
        sw->minute,
        sw->second,
        sw->gps_fix_valid
    );

    // Transmit (blocking here; you can switch to DMA if you like)
    HAL_UART_Transmit(huart, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

// Transmit just the screen state, e.g. when a button interrupt changes it
void SendScreenState(UART_HandleTypeDef *huart, UI_Screen_State_t state)
{
    char buf[32];
    int len = snprintf(buf, sizeof(buf),
        "SCREEN_STATE=%u\r\n",
        (unsigned)state
    );
    HAL_UART_Transmit(huart, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

//------------------------------------------------------------------------------
// Example: call SendSmartWatchData() periodically in your main loop or scheduler
//------------------------------------------------------------------------------
void Sensor_SmartWatch_log(const SmartWatchData_t *sw)
{
    SendSmartWatchData(&STLINK_UART, sw);
}



