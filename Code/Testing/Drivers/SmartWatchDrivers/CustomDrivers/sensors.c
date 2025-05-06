/*
 * sensors.c
 *
 *  Created on: May 5, 2025
 *      Author: exysta
 */
#include "MPU6500_driver.h"
#include "sensors.h"
#include "common_defs.h"

SmartWatchData_t SmartWatch_data_handle;

void read_mpu_data()
{
#define MAX_SAMPLES 10 // Max samples to read in one go (especially for FIFO)
	int16_t accel_raw[MAX_SAMPLES][3];
	float accel_g[MAX_SAMPLES][3];
	int16_t gyro_raw[MAX_SAMPLES][3];
	float gyro_dps[MAX_SAMPLES][3];
	uint16_t samples_read = MAX_SAMPLES; // Request up to MAX_SAMPLES
	uint8_t status;
	int i;

	status = mpu6500_read_hal(&MPU6500_I2C, MPU6500_I2C_ADDR_SHIFTED,
			HAL_MAX_DELAY, accel_raw, accel_g, gyro_raw, gyro_dps,
			&samples_read);

	SmartWatch_data_handle.accel_g[0];

	if (status == MPU6500_OK)
	{
		printf("Read %u samples successfully.\r\n", samples_read);
		for (i = 0; i < samples_read; i++)
		{
			printf("Sample %d:\r\n", i);
			printf("  Accel Raw:  X=%d, Y=%d, Z=%d\r\n", accel_raw[i][0],
					accel_raw[i][1], accel_raw[i][2]);
			printf("  Accel (g):  X=%.3f, Y=%.3f, Z=%.3f\r\n", accel_g[i][0],
					accel_g[i][1], accel_g[i][2]);
			printf("  Gyro Raw:   X=%d, Y=%d, Z=%d\r\n", gyro_raw[i][0],
					gyro_raw[i][1], gyro_raw[i][2]);
			printf("  Gyro (dps): X=%.2f, Y=%.2f, Z=%.2f\r\n", gyro_dps[i][0],
					gyro_dps[i][1], gyro_dps[i][2]);
		}
	}
	else
	{
		printf("MPU6500 read failed with status: %u\r\n", status);
	}
}
