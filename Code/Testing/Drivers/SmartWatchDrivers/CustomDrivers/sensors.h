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
#define MAX30102_WINDOW_SIZE   200  // Desired analysis window length (e.g., 200 samples for 2s @ 100Hz)

#define MAX30102_SAMPLES_PER_INTERRUPT 17   // Number of new samples you get each time
#define MAX30102_N_SAMPLES MAX30102_SAMPLES_PER_INTERRUPT // Process only new samples
#define MAX30102_PROCESSING_INTERVAL 200  // Process every 200ms

#define MAX30102_RAW_SAMPLING_RATE      800.0f // As configured in Sensor_MAX30102_configure_optimal_hr_spo2
#define MAX30102_AVERAGING_FACTOR     8      // As configured in Sensor_MAX30102_configure_optimal_hr_spo2 (for max30102_smp_ave_8)

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
void Sensor_MAX30102_init(max30102_t *obj, I2C_HandleTypeDef *hi2c);


// retrieve last estimates
uint8_t Sensor_MAX30102_get_hr(void);    // beats per minute
uint8_t Sensor_MAX30102_get_spo2(void);  // percent (0–100)


static uint8_t _detect_MAX30102_hr_N(const uint32_t *ir_buf, int num_samples_to_process);
static void _calc_acdc_N(const uint32_t *buf, int num_samples_to_process, float *ac, float *dc);

#endif /* SMARTWATCHDRIVERS_CUSTOMDRIVERS_SENSORS_H_ */
