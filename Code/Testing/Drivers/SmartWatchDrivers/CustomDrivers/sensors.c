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


// internal storage
static float MAX30102_fs;         // sampling rate in Hz
static uint8_t last_MAX30102_hr;
static uint8_t last_MAX30102_spo2;

// Application-level buffers for the full analysis window
static uint32_t app_ir_analysis_buffer[MAX30102_WINDOW_SIZE];
static uint32_t app_red_analysis_buffer[MAX30102_WINDOW_SIZE];
static int app_buffer_fill_count = 0;    // Current number of samples in our app_buffers

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
	Sensor_MAX30102_init(&SmartWatchData->max30102, &MAX30102_I2C); //800f is default sampling rate for config
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
        max30102_interrupt_handler(&sw->max30102); // This populates sw->max30102._ir_samples etc.

//        for (int i = 0; i < MAX30102_SAMPLES_PER_INTERRUPT; ++i)
//        {
//            if (app_buffer_fill_count < MAX30102_WINDOW_SIZE)
//            {
//                app_ir_analysis_buffer[app_buffer_fill_count] = sw->max30102._ir_samples[i];
//                app_red_analysis_buffer[app_buffer_fill_count] = sw->max30102._red_samples[i];
//
//                if(sw->max30102._ir_samples[i] == 0)
//                {
//                    printf("SAMPLE IR[%d] is 0 \r\n",i, sw->max30102._ir_samples[i]);
//
//                }
//                app_buffer_fill_count++;
//            }
//            else
//            {
//                // This case should ideally not be hit if we process when full.
//                // If it is, it means we missed a processing cycle or logic error.
//                // For now, we'll just break and process what we have.
//                // A more robust system might use a circular buffer for app_buffers.
//                printf("WARN: App buffer was already full when trying to add more samples.\r\n");
//                break;
//            }
//        }
//
//        // Do we have enough samples in our application buffer to perform analysis?
//        if (app_buffer_fill_count >= MAX30102_WINDOW_SIZE)
//        {
//            // printf("DEBUG: App buffer full. Processing %d samples.\n", APP_MAX30102_WINDOW_SIZE);
//            // printf("RAW APP IR (first 5): %lu %lu %lu %lu %lu\n",
//            //        app_ir_analysis_buffer[0], app_ir_analysis_buffer[1], app_ir_analysis_buffer[2],
//            //        app_ir_analysis_buffer[3], app_ir_analysis_buffer[4]);
//
//            float ir_ac, ir_dc, red_ac, red_dc;
//            _calc_acdc_N(app_ir_analysis_buffer, MAX30102_WINDOW_SIZE, &ir_ac, &ir_dc);
//            _calc_acdc_N(app_red_analysis_buffer, MAX30102_WINDOW_SIZE, &red_ac, &red_dc);
//
//            // printf("ACDC - IR_AC:%.1f IR_DC:%.1f | RED_AC:%.1f RED_DC:%.1f\n", ir_ac, ir_dc, red_ac, red_dc);
//
//            float R = 0.0f;
//            // Add more robust checks for AC/DC values before calculating R
//            if (ir_dc > 100.0f && red_dc > 100.0f && ir_ac > 10.0f && red_ac > 1.0f) { // Adjust thresholds as needed
//                R = (red_ac / red_dc) / (ir_ac / ir_dc);
//            } else {
//                // printf("WARN: Low AC/DC for SpO2. IR_AC:%.1f IR_DC:%.1f RED_AC:%.1f RED_DC:%.1f\n", ir_ac, ir_dc, red_ac, red_dc);
//            }
//
//            float MAX30102_spo2f = 110.0f - 25.0f * R; // Constants might need tuning
//            if (MAX30102_spo2f > 100.0f) MAX30102_spo2f = 100.0f;
//            else if (MAX30102_spo2f < 70.0f) MAX30102_spo2f = 70.0f; // Sensible lower physiological bound
//            last_MAX30102_spo2 = (uint8_t)MAX30102_spo2f;
//
//            last_MAX30102_hr = _detect_MAX30102_hr_N(app_ir_analysis_buffer, MAX30102_WINDOW_SIZE);
//
//            // Reset fill count for the next window of data
//            app_buffer_fill_count = 0;
//
//            // The printf in your original Sensor_max30102_Update will now use these new values
//             uint8_t bpm  = Sensor_MAX30102_get_hr(); // Gets last_MAX30102_hr
//             uint8_t spO2 = Sensor_MAX30102_get_spo2(); // Gets last_MAX30102_spo2
//            printf("hr : %u SPO2 : %u\r\n",bpm,spO2); // Added \n
//
//             //DEBUG
////             for(int i = 0; i < MAX30102_WINDOW_SIZE;i++)
////             {
////                 printf("ir : %d \r\n",app_ir_analysis_buffer[i]);
////                 printf("red : %d \r\n",app_red_analysis_buffer[i]);
////
////             }
//
//        }
    }
}

void max30102_plot(uint32_t ir_sample, uint32_t red_sample){
	 printf("ir : %d, red : %d \r\n",ir_sample,red_sample);
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
//	max30102_set_fifo_config(obj, max30102_smp_ave_8, 1, 7);
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
//	max30102_set_adc_resolution(obj, max30102_adc_8192);
//
//	// - Sampling Rate:
//	//   max30102_sr_100 (100 samples per second per active LED).
//	//   With no averaging (smp_ave_1), this gives an effective output rate of 100Hz.
//	max30102_set_sampling_rate(obj, max30102_sr_800);
//
//	// 6. Set LED Currents
//	// These are critical and likely need tuning for your specific setup.
//	// Start with moderate values (e.g., 6mA to 10mA).
//	// Example: 7.4 mA for both. Increase if PPG signal AC amplitude is too small.
//	// Decrease if DC level is saturating the ADC.
//	float led_current_ma = 7.0f; // Typical starting value in mA
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
//	//max30102_set_ppg_rdy(obj, 0); // Disable PPG_RDY interrupt (using A_FULL instead)
//	//max30102_set_alc_ovf(obj, 0); // Disable Ambient Light Cancellation Overflow interrupt
//	//max30102_set_die_temp_rdy(obj, 0);     // Disable Die Temp Ready interrupt
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
static void _calc_acdc_N(const uint32_t *buf, int num_samples_to_process, float *ac, float *dc)
{
    if (num_samples_to_process == 0) {
        *ac = 0.0f; *dc = 0.0f;
        return;
    }
    uint32_t mx = buf[0], mn = buf[0];
    uint64_t sum = buf[0]; // Initialize sum with the first element
    for (int i = 1; i < num_samples_to_process; i++) // Loop from 1 to num_samples_to_process
    {
        uint32_t v = buf[i];
        if (v > mx) mx = v;
        if (v < mn) mn = v;
        sum += v;
    }
    *ac = (float)(mx - mn);
    *dc = (float)sum / num_samples_to_process;
}

// very crude peak‐count on IR
static uint8_t _detect_MAX30102_hr_N(const uint32_t *ir_buf, int num_samples_to_process)
{
    // MAX30102_fs is the global effective sample rate
    if (num_samples_to_process < 3 || MAX30102_fs < 1.0f) { // Need at least 3 samples for peak detection logic
        return 0;
    }

    uint32_t mx = ir_buf[0], mn = ir_buf[0];
    for (int i = 1; i < num_samples_to_process; i++)
    {
        if (ir_buf[i] > mx) mx = ir_buf[i];
        if (ir_buf[i] < mn) mn = ir_buf[i];
    }

    // Print for debugging this specific function call
    // printf("HR_N: mx=%lu, mn=%lu, num_samples=%d, fs=%.1f\n", mx, mn, num_samples_to_process, MAX30102_fs);

    if (mx == mn) { // Flat line or all same values
        // printf("HR_N: Flat line (mx==mn), peaks=0\n");
        return 0;
    }

    float threshold = (mx + mn) * 0.5f;
    int peaks = 0;
    for (int i = 1; i < num_samples_to_process - 1; i++) // Iterate up to one before the last
    {
        if (ir_buf[i] > threshold && ir_buf[i - 1] <= threshold) {
            peaks++;
        }
    }

    // printf("HR_N: threshold=%.1f, peaks_found=%d\n", threshold, peaks);

    float window_sec = (float)num_samples_to_process / MAX30102_fs;
    if (window_sec < 0.1f) { // Avoid division by zero or too short window for meaningful BPM
        // printf("HR_N: Window too short (%.2f s), returning 0 BPM\n", window_sec);
        return 0;
    }

    uint8_t bpm = (uint8_t)((peaks * 60.0f) / window_sec);
    // printf("HR_N: Calculated BPM = %u (before sanity check)\n", bpm);

    return (bpm > 30 && bpm < 240) ? bpm : 0; // Basic sanity check
}

void Sensor_MAX30102_init(max30102_t *obj, I2C_HandleTypeDef *hi2c) // Removed sampling_rate_hz parameter
{
    MAX30102_fs = MAX30102_RAW_SAMPLING_RATE / MAX30102_AVERAGING_FACTOR;
    printf("DEBUG: Effective MAX30102_fs set to: %.2f Hz\r\n", MAX30102_fs);

    last_MAX30102_hr = 0;
    last_MAX30102_spo2 = 0;
    app_buffer_fill_count = 0;

    Sensor_MAX30102_configure_optimal_hr_spo2(obj, hi2c);
}



uint8_t Sensor_MAX30102_get_hr(void)
{
	return last_MAX30102_hr;
}

uint8_t Sensor_MAX30102_get_spo2(void)
{
	return last_MAX30102_spo2;
}
