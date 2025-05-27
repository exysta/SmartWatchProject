/*
 * common_defs.h
 *
 *  Created on: May 5, 2025
 *      Author: exysta
 */

#ifndef INC_COMMON_DEFS_H_
#define INC_COMMON_DEFS_H_

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include "MPU6500_driver.h" // Include sensor-specific data types if needed
#include "GNSS.h"
#include "i2c.h"
#include "usart.h"
#include "bmp280.h"
#include "max30102_for_stm32_hal.h"
//#define BLE_TEST
//#define SCREEN_TEST
#define BME280
//#define GPS_TEST
//#define MPU6500_TEST
//#define MAX30102_TEST

#define STLINK_UART huart3

#define BLE_UART huart4
#define BME280_I2C hi2c2
#define GNSS_UART huart5
#define MPU6500_I2C hi2c4
#define MAX30102_I2C hi2c1

#define BME280_ADDR (0x76 << 1)
#define RX_BUFFER_SIZE 256

#define MPU6500_I2C_ADDR_SHIFTED            (0x68 << 1)  // Shifted left for HAL compatibility
#define MPU6500_RGSTR_WHO_AM_I 0x75
#define MPU6500_MAX_SAMPLES 10 // sample stocked in memory for IMU data

// Enum to represent different screens/views
typedef enum {
    SCREEN_CLOCK,
    SCREEN_ENVIRONMENTAL, // Temp/Pressure/Humidity
    SCREEN_MOTION,       // Accel/Gyro
    SCREEN_HEART_RATE,   // HR/SpO2
    SCREEN_GPS_STATUS,
    // Add more screens as needed
    NUM_SCREENS // Keep this last for easy iteration
} UI_Screen_State_t;

// Structure to hold all relevant sensor data
typedef struct {
    // BMP280 Data
	BMP280_HandleTypedef bmp280;
    float temperature;
    float pressure; //in PA
    float humidity; //
    int bmp_data_valid;

    // MPU6500 Data
    //x = accel_g[0],y = accel_g[0],z = accel_g[2]
	float accel_g[MPU6500_MAX_SAMPLES][3];
	float gyro_dps[MPU6500_MAX_SAMPLES][3];

    int mpu_data_valid;

    // MAX30102 Data
    max30102_t max30102;
    int16_t heart_rate;
    float spo2;

    // GNSS Data (using struct from GNSS.h)
    GNSS_StateHandle gps_data;
    uint32_t         GNSS_Timer;

    // higher‐level outputs (add these):
    float    latitude;
    float    longitude;
    float    altitude;
    uint16_t year, month, day;
    uint8_t  hour, minute, second;
    uint8_t  gps_fix_valid;
    // in your SmartWatchData_t struct
    uint32_t gnss_nextRequestTick;
    uint8_t  gnss_state;      // 0 = idle, 1 = waiting for reply
} SmartWatchData_t;


#endif /* INC_COMMON_DEFS_H_ */
