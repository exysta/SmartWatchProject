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

// size taken from your library
#define MAX30102_N_SAMPLES  MAX30102_SAMPLE_LEN_MAX

// internal storage
static float MAX30102_fs;         // sampling rate in Hz
static uint8_t last_MAX30102_hr;
static uint8_t last_MAX30102_spo2;

void Sensor_SmartWatch_init(SmartWatchData_t *SmartWatchData)
{
    // 1) Zero entire struct to ensure all numeric fields are 0 / pointers NULL
    memset(SmartWatchData, 0, sizeof(*SmartWatchData));

    // 2) Explicitly set “valid” flags to false (0)
    SmartWatchData->bmp_data_valid = 0;
    SmartWatchData->mpu_data_valid = 0;

    // 3) Call sensor-specific init routines,
    Sensor_BMP280_init(&SmartWatchData->bmp280);                   // initialize BMP280 driver
    Sensor_GNSS_Init(SmartWatchData,&GNSS_UART);        // init GNSS data handle
	Sensor_MAX30102_init(800.0f, &SmartWatchData->max30102, &MAX30102_I2C); //800f is default sampling rate for config
}

void Sensor_SmartWatch_update(SmartWatchData_t *SmartWatchData)
{
  	Sensor_BMP280_read_data(SmartWatchData);
  	Sensor_GNSS_Update(SmartWatchData);
    Sensor_MPU6500_read_data(SmartWatchData);
    Sensor_max30102_Update(SmartWatchData);
}

//------------------------------------------------------------------------------
// Call this from your main loop (or a scheduler) to refresh GPS data at ~1 Hz
//------------------------------------------------------------------------------
void Sensor_max30102_Update(SmartWatchData_t *sw)
{
	if (max30102_has_interrupt(&sw->max30102))
	{
	  // Run interrupt handler to read FIFO
	  max30102_interrupt_handler(&sw->max30102);
	//	      // after handler, your heart._ir_samples and _red_samples are updated:
	//	      Sensor_MAX30102_compute(&max30102);
	//	      // now you can grab
	//	      uint8_t bpm  = Sensor_MAX30102_get_hr();
	//	      uint8_t spO2 = Sensor_MAX30102_get_spo2();
	//	      printf("hey %u %u",bpm,spO2);
	}
}



//------------------------------------------------------------------------------
// Call this once at startup to wire up the GNSS inside your SmartWatchData
//------------------------------------------------------------------------------
void Sensor_GNSS_Init(SmartWatchData_t *sw, UART_HandleTypeDef *huart)
{
    // 1) initialize the GNSS handle inside sw
    GNSS_Init(&sw->gps_data, huart);
    HAL_Delay(200);

    // 2) load your base configuration (turns off NMEA, enables UBX + Galileo)
    GNSS_LoadConfig(&sw->gps_data);
    HAL_Delay(200);

    // 3) apply the “stationary” dynamic model for best static accuracy
    GNSS_SetMode(&sw->gps_data, Stationary);
    HAL_Delay(100);

    // 4) seed your 1 Hz timer
    sw->GNSS_Timer = HAL_GetTick();

    // mark invalid until we get a fix
    sw->gps_data.fixType = 0;
}

//------------------------------------------------------------------------------
// Call this from your main loop (or a scheduler) to refresh GPS data at ~1 Hz
//------------------------------------------------------------------------------
void Sensor_GNSS_Update(SmartWatchData_t *sw)
{
    uint32_t now = HAL_GetTick();

    switch (sw->gnss_state) {
        case 0: // time to send a request at ~1 Hz?
            if (now >= sw->gnss_nextRequestTick) {
                GNSS_GetPVTData(&sw->gps_data);
                sw->gnss_state             = 1;
                sw->gnss_nextRequestTick   = now + 100;   // parse in 100 ms
            }
            break;

        case 1: // time to parse reply?
            if (now >= sw->gnss_nextRequestTick) {
                GNSS_ParseBuffer(&sw->gps_data);

                if (sw->gps_data.fixType >= 3) {
                    sw->latitude      = sw->gps_data.fLat;
                    sw->longitude     = sw->gps_data.fLon;
                    sw->altitude      = sw->gps_data.hMSL / 1000.0f;
                    sw->year          = sw->gps_data.year;
                    sw->month         = sw->gps_data.month;
                    sw->day           = sw->gps_data.day;
                    sw->hour          = sw->gps_data.hour;
                    sw->minute        = sw->gps_data.min;
                    sw->second        = sw->gps_data.sec;
                    sw->gps_fix_valid = 1;
                } else {
                    sw->gps_fix_valid = 0;
                }

                // go back to state 0 and schedule next request in 1 s
                sw->gnss_state           = 0;
                sw->gnss_nextRequestTick = now + 1000;
            }
            break;
    }
}

void Sensor_MPU6500_read_data(SmartWatchData_t *SmartWatchData_handle)
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
			SmartWatchData_handle->accel_g[i][0] = accel_g[i][0];
			SmartWatchData_handle->accel_g[i][1] = accel_g[i][1];
			SmartWatchData_handle->accel_g[i][2] = accel_g[i][2];

			SmartWatchData_handle->gyro_dps[i][0] = gyro_dps[i][0];
			SmartWatchData_handle->gyro_dps[i][1] = gyro_dps[i][1];
			SmartWatchData_handle->gyro_dps[i][2] = gyro_dps[i][2];
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

void Sensor_BMP280_init(BMP280_HandleTypedef *bmp280)
{
//	uint16_t size;
//	uint8_t txData = 0xD0;  // Array with a single element
//	// Command to send
//	uint8_t rxData[1];           // Buffer to store response
//	uint8_t Data[256];
//
//	HAL_I2C_Master_Transmit(&BME280_I2C, BME280_ADDR, &txData, 1, 100);
//	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&BME280_I2C, BME280_ADDR,
//			rxData, 1, 100);
//	printf("Received: 0x%02X \r\n", rxData[0]);  // Print response

	bmp280_init_default_params(&bmp280->params);
	bmp280->addr = BMP280_I2C_ADDRESS_0;
	bmp280->i2c = &BME280_I2C;

    // 3) try to initialize until the chip acks
    while (!bmp280_init(bmp280, &bmp280->params)) {
        // Optional: blink an LED, print a message, delay, etc.
        HAL_Delay(200);
    }
//	bool bme280p = bmp280.id == BME280_CHIP_ID;
//	size = sprintf((char*) Data, "BMP280: found %s \r\n",
//			bme280p ? "BME280" : "BMP280");
//	HAL_UART_Transmit(&STLINK_UART, Data, size, 1000);
}

void Sensor_BMP280_read_data(SmartWatchData_t *SmartWatchData)
{
	bmp280_read_float(&SmartWatchData->bmp280, &SmartWatchData->temperature,
			&SmartWatchData->pressure, &SmartWatchData->humidity);

}

/**
 * @brief Configures the MAX30102 sensor with generally optimal settings for MAX30102_hr and MAX30102_spo2.
 *
 * This function initializes the sensor, resets it, and sets various parameters
 * for typical photoplethysmography (PPG) applications. Temperature-related
 * features are not configured/enabled.
 *
 * @param obj Pointer to the max30102_t object instance.
 * @param hi2c Pointer to the I2C_HandleTypeDef for communication.
 *
 * Recommended effective sample rate: 100 Hz (100 IR samples, 100 RED samples per second)
 * Recommended ADC resolution: 18-bit
 *
 * Tune LED currents and ADC_RANGE based on observed signal quality.
 * Aim for DC values of raw signals to be ~25-75% of full ADC scale (0 to 262143 for 18-bit).
 * If signals are too low (small AC, low DC), increase LED current.
 * If signals are saturating (DC near max), decrease LED current or increase ADC range.
 */
void Sensor_MAX30102_configure_optimal_hr_spo2(max30102_t *obj,
		I2C_HandleTypeDef *hi2c)
{
//	// 1. Initialize the driver structure
//	max30102_init(obj, hi2c);
//
//	// 2. Reset the sensor to ensure a known state
//	// This also puts the sensor in shutdown mode and clears most registers.
//	max30102_reset(obj);
//	// A small delay might be good after reset, though not strictly specified for all operations.
//	// HAL_Delay(10); // Optional: if issues arise, consider a small delay.
//
//	// 3. Clear FIFO pointers and overflow counter
//	// (max30102_set_mode also clears FIFO, but explicit call here is good practice after reset)
//	max30102_clear_fifo(obj);
//
//	// 4. Configure FIFO
//	// - smp_ave: Sample averaging. max30102_smp_ave_1 means no averaging.
//	//   This gives the rawest data at the selected sample rate.
//	// - roll_over_en: 1 to enable roll-over (new data overwrites old when FIFO is full).
//	// - fifo_a_full: Interrupt when FIFO has (32 - N) samples. N=7 means interrupt at 25 samples.
//	max30102_set_fifo_config(obj, max30102_smp_ave_1, 1, 7);
//
//	// 5. Configure MAX30102_spo2 ADC and LED pulse characteristics
//	// - LED Pulse Width: Determines ADC resolution.
//	//   max30102_pw_18_bit (411µs) gives 18-bit ADC resolution.
//	max30102_set_led_pulse_width(obj, max30102_pw_18_bit);
//
//	// - ADC Range/Resolution:
//	//   max30102_adc_4096 (4096nA full scale) is a good starting point.
//	//   If saturation occurs (DC values near 2^18-1), consider max30102_adc_8192.
//	//   If signal is too weak, ensure LED currents are adequate before reducing range.
//	max30102_set_adc_resolution(obj, max30102_adc_4096);
//
//	// - Sampling Rate:
//	//   max30102_sr_100 (100 samples per second per active LED).
//	//   With no averaging (smp_ave_1), this gives an effective output rate of 100Hz.
//	max30102_set_sampling_rate(obj, max30102_sr_100);
//
//	// 6. Set LED Currents
//	// These are critical and likely need tuning for your specific setup.
//	// Start with moderate values (e.g., 6mA to 10mA).
//	// Example: 7.4 mA for both. Increase if PPG signal AC amplitude is too small.
//	// Decrease if DC level is saturating the ADC.
//	float led_current_ma = 7.4f; // Typical starting value in mA
//	max30102_set_led_current_1(obj, led_current_ma); // IR LED
//	max30102_set_led_current_2(obj, led_current_ma); // RED LED
//	// For multi-LED mode with more LEDs, you'd use max30102_set_multi_led_slot_x_y functions.
//	// For MAX30102_spo2 mode, LED_PA1 (IR) and LED_PA2 (RED) are automatically used.
//
//	// 7. Set the operating mode to MAX30102_spo2 mode
//	// This enables both IR and RED LEDs and begins measurements.
//	// This function also clears the FIFO.
//	max30102_set_mode(obj, max30102_spo2);
//
//	// 8. Enable Interrupts
//	// - A_FULL (FIFO Almost Full): Triggers when FIFO reaches the tMAX30102_hreshold set by fifo_a_full.
//	max30102_set_a_full(obj, 1);
//
//	// Ensure other (unused) interrupts are disabled.
//	// After reset, interrupt enable registers are 0x00, so they are already disabled.
//	// Explicitly ensuring they are off can be good for clarity if reset behavior is uncertain.
//	max30102_set_ppg_rdy(obj, 0); // Disable PPG_RDY interrupt (using A_FULL instead)
//	max30102_set_alc_ovf(obj, 0); // Disable Ambient Light Cancellation Overflow interrupt
//	max30102_set_die_temp_rdy(obj, 0);     // Disable Die Temp Ready interrupt
//
//	// 9. Ensure die temperature measurement is disabled (as per request)
//	// After reset, DIE_TEMP_CONFIG register is 0x00, so TEMP_EN is already 0.
//	max30102_set_die_temp_en(obj, 0);

	//------------------ DEBUG -----------------------
	// Initiation
	max30102_init(obj, hi2c);
	max30102_reset(obj);
	max30102_clear_fifo(obj);
	max30102_set_fifo_config(obj, max30102_smp_ave_8, 1, 7);

	// Sensor settings
	max30102_set_led_pulse_width(obj, max30102_pw_16_bit);
	max30102_set_adc_resolution(obj, max30102_adc_2048);
	max30102_set_sampling_rate(obj, max30102_sr_800);
	max30102_set_led_current_1(obj, 6.2);
	max30102_set_led_current_2(obj, 6.2);

	// Enter SpO2 mode
	max30102_set_mode(obj, max30102_spo2);
	max30102_set_a_full(obj, 1);

	// Initiate 1 temperature measurement
	max30102_set_die_temp_en(obj, 1);
	max30102_set_die_temp_rdy(obj, 1);

	uint8_t en_reg[2] =
	{ 0 };
	max30102_read(obj, 0x00, en_reg, 1);

	//Enter measurement mode:
	// Enter SpO2 mode
	max30102_set_mode(obj, max30102_spo2);

	//Enable the required interrupts:
	// Enable FIFO_A_FULL interrupt
	max30102_set_a_full(obj, 1);
	// Enable die temperature measurement
	max30102_set_die_temp_en(obj, 1);
	// Enable DIE_TEMP_RDY interrupt
	max30102_set_die_temp_rdy(obj, 1);
}

// helper: AC/DC on a buffer of uint32_t
static void _calc_acdc(const uint32_t *buf, float *ac, float *dc)
{
	uint32_t mx = buf[0], mn = buf[0];
	uint64_t sum = 0;
	for (int i = 0; i < MAX30102_N_SAMPLES; i++)
	{
		uint32_t v = buf[i];
		if (v > mx)
			mx = v;
		if (v < mn)
			mn = v;
		sum += v;
	}
	*ac = (float) (mx - mn);
	*dc = (float) sum / MAX30102_N_SAMPLES;
}

// very crude peak‐count on IR to get MAX30102_hr
static uint8_t _detect_MAX30102_hr(const uint32_t *ir)
{
	// tMAX30102_hreshold at midway
	uint32_t mx = ir[0], mn = ir[0];
	for (int i = 1; i < MAX30102_N_SAMPLES; i++)
	{
		if (ir[i] > mx)
			mx = ir[i];
		if (ir[i] < mn)
			mn = ir[i];
	}
	float tMAX30102_hr = (mx + mn) * 0.5f;

	int peaks = 0;
	for (int i = 1; i < MAX30102_N_SAMPLES - 1; i++)
	{
		if (ir[i] > tMAX30102_hr && ir[i - 1] <= tMAX30102_hr)
			peaks++;
	}
	// convert to BPM
	float window_sec = MAX30102_N_SAMPLES / MAX30102_fs;
	return (uint8_t) ((peaks * 60.0f) / window_sec);
}

void Sensor_MAX30102_init(float sampling_rate_hz, max30102_t *obj,
		I2C_HandleTypeDef *hi2c)
{
	MAX30102_fs = sampling_rate_hz;
	last_MAX30102_hr = 0;
	last_MAX30102_spo2 = 0;
	Sensor_MAX30102_configure_optimal_hr_spo2(obj,
			hi2c);
}

void Sensor_MAX30102_compute(max30102_t *sensor)
{
	// compute AC/DC for IR and Red
	float ir_ac, ir_dc, red_ac, red_dc;
	_calc_acdc(sensor->_ir_samples, &ir_ac, &ir_dc);
	_calc_acdc(sensor->_red_samples, &red_ac, &red_dc);

	// ratio‐of‐ratios for SpO₂
	float R = (red_ac / red_dc) / (ir_ac / ir_dc);
	// linear calibration — tweak these constants as needed
	float MAX30102_spo2f = 110.0f - 25.0f * R;
	if (MAX30102_spo2f > 100.0f)
		MAX30102_spo2f = 100.0f;
	else if (MAX30102_spo2f < 0.0f)
		MAX30102_spo2f = 0.0f;
	last_MAX30102_spo2 = (uint8_t) MAX30102_spo2f;

	// heart rate from IR
	last_MAX30102_hr = _detect_MAX30102_hr(sensor->_ir_samples);
}

uint8_t Sensor_MAX30102_get_hr(void)
{
	return last_MAX30102_hr;
}

uint8_t Sensor_MAX30102_get_spo2(void)
{
	return last_MAX30102_spo2;
}
