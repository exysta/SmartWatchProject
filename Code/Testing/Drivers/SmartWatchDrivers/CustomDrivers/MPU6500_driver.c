#include "MPU6500_driver.h"

/**
 * @brief  Read accelerometer and gyroscope data directly using HAL I2C.
 * @param  hi2c Pointer to the initialized HAL I2C handle (e.g., &hi2c1).
 * @param  i2c_addr_shifted The MPU6500's 8-bit I2C address (7-bit address << 1).
 * @param  timeout Timeout duration for I2C communication (e.g., HAL_MAX_DELAY or ms).
 * @param  accel_raw Pointer to an array buf to store the raw accel data [samples][x,y,z].
 * @param  accel_g Pointer to an array buf to store the accel data in g [samples][x,y,z].
 * @param  gyro_raw Pointer to an array buf to store the raw gyro data [samples][x,y,z].
 * @param  gyro_dps Pointer to an array buf to store the gyro data in dps [samples][x,y,z].
 * @param  len Input: Pointer to the maximum number of samples to read.
 *             Output: Pointer to the actual number of samples read.
 * @return status code (MPU6500_OK, MPU6500_ERR_READ, etc.)
 * @note   Reads sensor configuration to apply correct scaling.
 * @note   Reads multiple samples if FIFO is enabled and contains data.
 * @note   Uses a local stack buffer; ensure sufficient stack space (~1KB for FIFO).
 */
uint8_t mpu6500_read_hal(I2C_HandleTypeDef *hi2c, uint16_t i2c_addr_shifted, uint32_t timeout,
                         int16_t (*accel_raw)[3], float (*accel_g)[3],
                         int16_t (*gyro_raw)[3], float (*gyro_dps)[3],
                         uint16_t *len)
{
    HAL_StatusTypeDef hal_res;
    uint8_t user_ctrl_reg;
    uint8_t accel_conf_reg;
    uint8_t gyro_conf_reg;
    uint8_t accel_fs_sel;
    uint8_t gyro_fs_sel;

    // --- Internal buffer for I2C reads ---
    // Size needs to accommodate max FIFO read (1024 bytes) or normal read (14 bytes).
    // Using a large buffer on the stack can be risky on memory-constrained MCUs.
    uint8_t read_buffer[1024]; // Be mindful of stack usage!

    // --- Parameter Checks ---
    if (hi2c == NULL || accel_raw == NULL || accel_g == NULL ||
        gyro_raw == NULL || gyro_dps == NULL || len == NULL)
    {
        return MPU6500_ERR_PARAM; // return error: null pointer
    }
    if ((*len) == 0)
    {
        return MPU6500_ERR_LEN_ZERO; // return error: length is zero
    }

    // --- Read configurations using HAL ---
    hal_res = HAL_I2C_Mem_Read(hi2c, i2c_addr_shifted, MPU6500_REG_USER_CTRL,
                               I2C_MEMADD_SIZE_8BIT, &user_ctrl_reg, 1, timeout);
    if (hal_res != HAL_OK) { /* printf("Debug: Read USER_CTRL failed (%d)\n", hal_res); */ return MPU6500_ERR_READ; }

    hal_res = HAL_I2C_Mem_Read(hi2c, i2c_addr_shifted, MPU6500_REG_ACCEL_CONFIG,
                               I2C_MEMADD_SIZE_8BIT, &accel_conf_reg, 1, timeout);
    if (hal_res != HAL_OK) { /* printf("Debug: Read ACCEL_CONFIG failed (%d)\n", hal_res); */ return MPU6500_ERR_READ; }

    hal_res = HAL_I2C_Mem_Read(hi2c, i2c_addr_shifted, MPU6500_REG_GYRO_CONFIG,
                               I2C_MEMADD_SIZE_8BIT, &gyro_conf_reg, 1, timeout);
    if (hal_res != HAL_OK) { /* printf("Debug: Read GYRO_CONFIG failed (%d)\n", hal_res); */ return MPU6500_ERR_READ; }

    // Extract sensitivity settings (FS_SEL bits)
    accel_fs_sel = (accel_conf_reg >> 3) & 0x03;
    gyro_fs_sel  = (gyro_conf_reg >> 3) & 0x03;

    // --- Check if FIFO mode is enabled (USER_CTRL bit 6) ---
    if ((user_ctrl_reg & (1 << 6)) != 0)
    {
        // --- FIFO Mode ---
        uint8_t fifo_en_conf;
        uint8_t fifo_count_buf[2];
        uint16_t count_bytes;
        uint16_t num_samples;
        uint16_t i;

        // Read FIFO_EN register to verify expected configuration (Accel + Gyro)
        hal_res = HAL_I2C_Mem_Read(hi2c, i2c_addr_shifted, MPU6500_REG_FIFO_EN,
                                   I2C_MEMADD_SIZE_8BIT, &fifo_en_conf, 1, timeout);
        if (hal_res != HAL_OK) { /* printf("Debug: Read FIFO_EN failed (%d)\n", hal_res); */ return MPU6500_ERR_READ; }

        // Check if Accel (bit 3) and Gyro (bits 4,5,6) are enabled
        // 0x78 = 0b0111 1000 (Gyro X,Y,Z + Accel)
        if (fifo_en_conf != 0x78)
        {
            // Allow only accel+gyro data in FIFO for this specific implementation
             /* printf("Debug: Unexpected FIFO_EN config (0x%02X), expected 0x78\n", fifo_en_conf); */
            return MPU6500_ERR_FIFO_CONF;
        }

        // Read FIFO Count (High and Low bytes)
        hal_res = HAL_I2C_Mem_Read(hi2c, i2c_addr_shifted, MPU6500_REG_FIFO_COUNTH,
                                   I2C_MEMADD_SIZE_8BIT, fifo_count_buf, 2, timeout);
        if (hal_res != HAL_OK) { /* printf("Debug: Read FIFO_COUNT failed (%d)\n", hal_res); */ return MPU6500_ERR_READ; }

        count_bytes = (uint16_t)(((uint16_t)fifo_count_buf[0] << 8) | fifo_count_buf[1]);

        // Limit bytes to prevent overflow reads and check against internal buffer size
        if (count_bytes > sizeof(read_buffer)) {
            /* printf("Debug: Warning - FIFO count (%d) > buffer size (%d). Limiting.\n", count_bytes, sizeof(read_buffer)); */
            count_bytes = sizeof(read_buffer);
        }
        // Datasheet implies max 1024, but checking against buffer is safer
        if (count_bytes >= 1024) {
             /* printf("Debug: Warning - FIFO count near/at max (%d bytes).\n", count_bytes); */
             // Note: If overflow truly happened *before* this read, data might be corrupted.
             // Consider adding FIFO reset logic here or in initialization if overflows are expected.
             count_bytes = 1024; // Cap at theoretical max / buffer size
        }


        // Calculate how many samples fit in the available bytes (12 bytes per sample)
        uint16_t available_samples = count_bytes / 12;
        uint16_t requested_max_samples = (*len);

        num_samples = available_samples;
        if (num_samples > requested_max_samples) {
            num_samples = requested_max_samples; // Limit by user request
        }

        count_bytes = num_samples * 12; // Final number of bytes to read

        if (num_samples == 0) {
            *len = 0; // No complete samples available or requested
            return MPU6500_OK; // Not an error, just no data
        }

        // Read the data from FIFO register (MPU6500_REG_R_W) into the local buffer
        hal_res = HAL_I2C_Mem_Read(hi2c, i2c_addr_shifted, MPU6500_REG_R_W,
                                   I2C_MEMADD_SIZE_8BIT, read_buffer, count_bytes, timeout);
        if (hal_res != HAL_OK)
        {
             /* printf("Debug: Read FIFO data failed (%d)\n", hal_res); */
            *len = 0; // Indicate no data was successfully read
            return MPU6500_ERR_READ;
        }

        // --- Process the read FIFO data ---
        *len = num_samples; // Update the output length
        for (i = 0; i < num_samples; i++)
        {
            // Extract raw values (Big Endian) from the local buffer
            accel_raw[i][0] = (int16_t)(((uint16_t)read_buffer[i * 12 + 0] << 8) | read_buffer[i * 12 + 1]);
            accel_raw[i][1] = (int16_t)(((uint16_t)read_buffer[i * 12 + 2] << 8) | read_buffer[i * 12 + 3]);
            accel_raw[i][2] = (int16_t)(((uint16_t)read_buffer[i * 12 + 4] << 8) | read_buffer[i * 12 + 5]);
            gyro_raw[i][0]  = (int16_t)(((uint16_t)read_buffer[i * 12 + 6] << 8) | read_buffer[i * 12 + 7]);
            gyro_raw[i][1]  = (int16_t)(((uint16_t)read_buffer[i * 12 + 8] << 8) | read_buffer[i * 12 + 9]);
            gyro_raw[i][2]  = (int16_t)(((uint16_t)read_buffer[i * 12 + 10] << 8) | read_buffer[i * 12 + 11]);

            // --- Convert raw values to physical units ---
            // Accel conversion
            if (accel_fs_sel == 0) { // ±2g
                accel_g[i][0] = (float)(accel_raw[i][0]) / 16384.0f;
                accel_g[i][1] = (float)(accel_raw[i][1]) / 16384.0f;
                accel_g[i][2] = (float)(accel_raw[i][2]) / 16384.0f;
            } else if (accel_fs_sel == 1) { // ±4g
                accel_g[i][0] = (float)(accel_raw[i][0]) / 8192.0f;
                accel_g[i][1] = (float)(accel_raw[i][1]) / 8192.0f;
                accel_g[i][2] = (float)(accel_raw[i][2]) / 8192.0f;
            } else if (accel_fs_sel == 2) { // ±8g
                accel_g[i][0] = (float)(accel_raw[i][0]) / 4096.0f;
                accel_g[i][1] = (float)(accel_raw[i][1]) / 4096.0f;
                accel_g[i][2] = (float)(accel_raw[i][2]) / 4096.0f;
            } else { // ±16g
                accel_g[i][0] = (float)(accel_raw[i][0]) / 2048.0f;
                accel_g[i][1] = (float)(accel_raw[i][1]) / 2048.0f;
                accel_g[i][2] = (float)(accel_raw[i][2]) / 2048.0f;
            }

            // Gyro conversion
            if (gyro_fs_sel == 0) { // ±250 dps
                gyro_dps[i][0] = (float)(gyro_raw[i][0]) / 131.0f;
                gyro_dps[i][1] = (float)(gyro_raw[i][1]) / 131.0f;
                gyro_dps[i][2] = (float)(gyro_raw[i][2]) / 131.0f;
            } else if (gyro_fs_sel == 1) { // ±500 dps
                gyro_dps[i][0] = (float)(gyro_raw[i][0]) / 65.5f;
                gyro_dps[i][1] = (float)(gyro_raw[i][1]) / 65.5f;
                gyro_dps[i][2] = (float)(gyro_raw[i][2]) / 65.5f;
            } else if (gyro_fs_sel == 2) { // ±1000 dps
                gyro_dps[i][0] = (float)(gyro_raw[i][0]) / 32.8f;
                gyro_dps[i][1] = (float)(gyro_raw[i][1]) / 32.8f;
                gyro_dps[i][2] = (float)(gyro_raw[i][2]) / 32.8f;
            } else { // ±2000 dps
                gyro_dps[i][0] = (float)(gyro_raw[i][0]) / 16.4f;
                gyro_dps[i][1] = (float)(gyro_raw[i][1]) / 16.4f;
                gyro_dps[i][2] = (float)(gyro_raw[i][2]) / 16.4f;
            }
        }
        return MPU6500_OK; // success
    }
    else
    {
        // --- Normal Mode (Read directly from sensor registers) ---
        *len = 1; // Normal mode always reads one sample

        // Read 14 bytes starting from ACCEL_XOUT_H into the local buffer
        // ACCEL_X/Y/Z, TEMP, GYRO_X/Y/Z (each 2 bytes)
        hal_res = HAL_I2C_Mem_Read(hi2c, i2c_addr_shifted, MPU6500_REG_ACCEL_XOUT_H,
                                   I2C_MEMADD_SIZE_8BIT, read_buffer, 14, timeout);
        if (hal_res != HAL_OK)
        {
             /* printf("Debug: Read sensor data failed (%d)\n", hal_res); */
            *len = 0; // Indicate no data read
            return MPU6500_ERR_READ;
        }

        // --- Process the read sensor data ---
        // Extract raw values (Big Endian) from the local buffer
        accel_raw[0][0] = (int16_t)(((uint16_t)read_buffer[0] << 8) | read_buffer[1]);
        accel_raw[0][1] = (int16_t)(((uint16_t)read_buffer[2] << 8) | read_buffer[3]);
        accel_raw[0][2] = (int16_t)(((uint16_t)read_buffer[4] << 8) | read_buffer[5]);
        // Skip Temperature bytes read_buffer[6] and read_buffer[7]
        gyro_raw[0][0] = (int16_t)(((uint16_t)read_buffer[8] << 8) | read_buffer[9]);
        gyro_raw[0][1] = (int16_t)(((uint16_t)read_buffer[10] << 8) | read_buffer[11]);
        gyro_raw[0][2] = (int16_t)(((uint16_t)read_buffer[12] << 8) | read_buffer[13]);

        // --- Convert raw values to physical units ---
        // Accel conversion
        if (accel_fs_sel == 0) { // ±2g
            accel_g[0][0] = (float)(accel_raw[0][0]) / 16384.0f;
            accel_g[0][1] = (float)(accel_raw[0][1]) / 16384.0f;
            accel_g[0][2] = (float)(accel_raw[0][2]) / 16384.0f;
        } else if (accel_fs_sel == 1) { // ±4g
            accel_g[0][0] = (float)(accel_raw[0][0]) / 8192.0f;
            accel_g[0][1] = (float)(accel_raw[0][1]) / 8192.0f;
            accel_g[0][2] = (float)(accel_raw[0][2]) / 8192.0f;
        } else if (accel_fs_sel == 2) { // ±8g
            accel_g[0][0] = (float)(accel_raw[0][0]) / 4096.0f;
            accel_g[0][1] = (float)(accel_raw[0][1]) / 4096.0f;
            accel_g[0][2] = (float)(accel_raw[0][2]) / 4096.0f;
        } else { // ±16g
            accel_g[0][0] = (float)(accel_raw[0][0]) / 2048.0f;
            accel_g[0][1] = (float)(accel_raw[0][1]) / 2048.0f;
            accel_g[0][2] = (float)(accel_raw[0][2]) / 2048.0f;
        }

        // Gyro conversion
        if (gyro_fs_sel == 0) { // ±250 dps
            gyro_dps[0][0] = (float)(gyro_raw[0][0]) / 131.0f;
            gyro_dps[0][1] = (float)(gyro_raw[0][1]) / 131.0f;
            gyro_dps[0][2] = (float)(gyro_raw[0][2]) / 131.0f;
        } else if (gyro_fs_sel == 1) { // ±500 dps
            gyro_dps[0][0] = (float)(gyro_raw[0][0]) / 65.5f;
            gyro_dps[0][1] = (float)(gyro_raw[0][1]) / 65.5f;
            gyro_dps[0][2] = (float)(gyro_raw[0][2]) / 65.5f;
        } else if (gyro_fs_sel == 2) { // ±1000 dps
            gyro_dps[0][0] = (float)(gyro_raw[0][0]) / 32.8f;
            gyro_dps[0][1] = (float)(gyro_raw[0][1]) / 32.8f;
            gyro_dps[0][2] = (float)(gyro_raw[0][2]) / 32.8f;
        } else { // ±2000 dps
            gyro_dps[0][0] = (float)(gyro_raw[0][0]) / 16.4f;
            gyro_dps[0][1] = (float)(gyro_raw[0][1]) / 16.4f;
            gyro_dps[0][2] = (float)(gyro_raw[0][2]) / 16.4f;
        }

        return MPU6500_OK; // success
    }
}
