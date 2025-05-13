/*
 * sensors.h
 *
 *  Created on: May 5, 2025
 *      Author: exysta
 */

#ifndef SMARTWATCHDRIVERS_CUSTOMDRIVERS_SENSORS_H_
#define SMARTWATCHDRIVERS_CUSTOMDRIVERS_SENSORS_H_

#include "max30102_for_stm32_hal.h"
// Define buffer sizes
#define MAX30102_BUFFER_SIZE 300  // 3 seconds at 100Hz
#define MAX30102_PROCESSING_INTERVAL 200  // Process every 200ms

void Sensor_BMP280_read_data();
void Sensor_BMP280_init();
void Sensor_MPU6500_read_data();



void Sensor_MAX30102_configure_optimal_hr_spo2(max30102_t *obj,
		I2C_HandleTypeDef *hi2c);
// call once after you initialize your max30102_t
//   sr_hz = your configured sampling rate in Hertz (e.g. 100.0f)
void Sensor_MAX30102_init(float sr_hz);

// run this after each time you call max30102_read_fifo(&sensor),
// so that sensor._ir_samples[] / _red_samples[] are fresh
void Sensor_MAX30102_compute(max30102_t *sensor);

// retrieve last estimates
uint8_t Sensor_MAX30102_get_hr(void);    // beats per minute
uint8_t Sensor_MAX30102_get_spo2(void);  // percent (0–100)

#endif /* SMARTWATCHDRIVERS_CUSTOMDRIVERS_SENSORS_H_ */
