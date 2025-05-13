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

#define BLE_TEST
//#define SCREEN_TEST
//#define BME280
//#define GPS_TEST
//#define MPU6500_TEST
//#define MAX30102_TEST

#define STLINK_UART huart4

#define BLE_UART huart1
#define BME280_I2C hi2c2
#define GNSS_UART huart3
#define MPU6500_I2C hi2c1
#define MAX30102_I2C hi2c3
#define ST7789_SPI_PORT hspi2

#define ST7789_BLK_PIN  ST7789_BLK_Pin

#define BME280_ADDR 0x76
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
    int16_t heart_rate;
    float spo2;

    // GNSS Data (using struct from GNSS.h)
    GNSS_StateHandle gps_data;
    // bool gps_fix_valid; // This might be within GPS_t already

} SmartWatchData_t;

#endif /* INC_COMMON_DEFS_H_ */
