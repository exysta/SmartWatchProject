/*
 * sensors.h
 *
 *  Created on: May 5, 2025
 *      Author: exysta
 */

#ifndef SMARTWATCHDRIVERS_CUSTOMDRIVERS_SENSORS_H_
#define SMARTWATCHDRIVERS_CUSTOMDRIVERS_SENSORS_H_

#include "max30102_for_stm32_hal.h"
#include "common_defs.h"

// Define buffer sizes
#define MAX30102_BUFFER_SIZE 300  // 3 seconds at 100Hz
#define MAX30102_PROCESSING_INTERVAL 200  // Process every 200ms

void Sensor_SmartWatch_init(SmartWatchData_t *SmartWatchData);
void Sensor_SmartWatch_update(SmartWatchData_t *SmartWatchData);

void Sensor_BMP280_read_data(SmartWatchData_t *SmartWatchData);
void Sensor_BMP280_init(BMP280_HandleTypedef *bmp280);

void Sensor_max30102_Update(SmartWatchData_t *sw);


void Sensor_MPU6500_read_data(SmartWatchData_t *SmartWatchData_handle);


void Sensor_GNSS_Init(SmartWatchData_t *sw, UART_HandleTypeDef *huart);
void Sensor_GNSS_Update(SmartWatchData_t *sw);


void Sensor_MAX30102_configure_optimal_hr_spo2(max30102_t *obj,
		I2C_HandleTypeDef *hi2c);
// call once after you initialize your max30102_t
//   sr_hz = your configured sampling rate in Hertz (e.g. 100.0f)
void Sensor_MAX30102_init(float sampling_rate_hz, max30102_t *obj,
		I2C_HandleTypeDef *hi2c);
// run this after each time you call max30102_read_fifo(&sensor),
// so that sensor._ir_samples[] / _red_samples[] are fresh
void Sensor_MAX30102_compute(max30102_t *sensor);

// retrieve last estimates
uint8_t Sensor_MAX30102_get_hr(void);    // beats per minute
uint8_t Sensor_MAX30102_get_spo2(void);  // percent (0–100)

#endif /* SMARTWATCHDRIVERS_CUSTOMDRIVERS_SENSORS_H_ */
