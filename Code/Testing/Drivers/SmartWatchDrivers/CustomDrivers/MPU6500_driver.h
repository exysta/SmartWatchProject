/*
 * MPU6500_driver.h
 *
 *  Created on: Apr 18, 2025
 *      Author: exysta
 */

#ifndef INC_MPU6500_DRIVER_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stddef.h> // For NULL check

// --- MPU6500 Register Definitions (Ensure these are defined somewhere) ---
#define MPU6500_REG_USER_CTRL        0x6A
#define MPU6500_REG_ACCEL_CONFIG     0x1C
#define MPU6500_REG_GYRO_CONFIG      0x1B
#define MPU6500_REG_FIFO_EN          0x23
#define MPU6500_REG_FIFO_COUNTH      0x72
#define MPU6500_REG_R_W              0x74 // FIFO Read/Write Register
#define MPU6500_REG_ACCEL_XOUT_H     0x3B
// Add other registers if needed by initialization code, but not strictly by this read function.

// --- Return Status Codes ---
#define MPU6500_OK              0  // Success
#define MPU6500_ERR_READ        1  // HAL I2C Read Error
#define MPU6500_ERR_PARAM       2  // Null pointer or invalid parameter
#define MPU6500_ERR_LEN_ZERO    4  // Requested length (*len) is zero
#define MPU6500_ERR_FIFO_CONF   6  // Unexpected FIFO_EN register value

#define INC_MPU6500_DRIVER_H_

uint8_t mpu6500_read_hal(I2C_HandleTypeDef *hi2c, uint16_t i2c_addr_shifted, uint32_t timeout,
                         int16_t (*accel_raw)[3], float (*accel_g)[3],
                         int16_t (*gyro_raw)[3], float (*gyro_dps)[3],
                         uint16_t *len);

#endif /* INC_MPU6500_DRIVER_H_ */
