/*
 * sensors.c
 *
 *  Created on: May 5, 2025
 *      Author: exysta
 */
#include "MPU6500_driver.h"
#include "bmp280.h"

#include "sensors.h"
#include "common_defs.h"
#include <stdio.h>

SmartWatchData_t SmartWatchData_handle;
BMP280_HandleTypedef bmp280;

void Sensor_MPU6500_read_data()
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
	HAL_MAX_DELAY, accel_raw, accel_g, gyro_raw, gyro_dps, &samples_read);

	if (status == MPU6500_OK)
	{
		for (i = 0; i < samples_read; i++)
		{
			SmartWatchData_handle.accel_g[i][0] = accel_g[i][0];
			SmartWatchData_handle.accel_g[i][0] = accel_g[i][0];
			SmartWatchData_handle.accel_g[i][0] = accel_g[i][0];

			SmartWatchData_handle.gyro_dps[i][0] = gyro_dps[i][0];
			SmartWatchData_handle.gyro_dps[i][0] = gyro_dps[i][0];
			SmartWatchData_handle.gyro_dps[i][0] = gyro_dps[i][0];
		}
		//DEBUG
//		printf("Read %u samples successfully.\r\n", samples_read);
//		for (i = 0; i < samples_read; i++)
//		{
//			printf("Sample %d:\r\n", i);
//			printf("  Accel Raw:  X=%d, Y=%d, Z=%d\r\n", accel_raw[i][0],
//					accel_raw[i][1], accel_raw[i][2]);
//			printf("  Accel (g):  X=%.3f, Y=%.3f, Z=%.3f\r\n", accel_g[i][0],
//					accel_g[i][1], accel_g[i][2]);
//			printf("  Gyro Raw:   X=%d, Y=%d, Z=%d\r\n", gyro_raw[i][0],
//					gyro_raw[i][1], gyro_raw[i][2]);
//			printf("  Gyro (dps): X=%.2f, Y=%.2f, Z=%.2f\r\n", gyro_dps[i][0],
//					gyro_dps[i][1], gyro_dps[i][2]);
//		}
	}
	else
	{
		printf("MPU6500 read failed with status: %u\r\n", status);
	}
}

void Sensor_BMP280_init()
{
	uint16_t size;
	uint8_t txData = 0xD0;  // Array with a single element
	// Command to send
	uint8_t rxData[1];           // Buffer to store response
	uint8_t Data[256];

	HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, &txData, 1, 100);
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&BME280_I2C, BME280_ADDR,
			rxData, 1, 100);
	printf("Received: 0x%02X \r\n", rxData[0]);  // Print response

	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &BME280_I2C;

	while (!bmp280_init(&bmp280, &bmp280.params))
	{
		size = sprintf((char*) Data, "BMP280 initialization failed\r\n");
		HAL_UART_Transmit(&STLINK_UART, Data, size, 1000);
		HAL_Delay(2000);
	}
	bool bme280p = bmp280.id == BME280_CHIP_ID;
	size = sprintf((char*) Data, "BMP280: found %s \r\n",
			bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&STLINK_UART, Data, size, 1000);
}

void Sensor_BMP280_read_data()
{
	bmp280_read_float(&bmp280, &SmartWatchData_handle.temperature, &SmartWatchData_handle.pressure, &SmartWatchData_handle.humidity);

}

