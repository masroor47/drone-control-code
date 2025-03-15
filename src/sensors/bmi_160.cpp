#include "esp_log.h"
#include "sensors/bmi_160.hpp"


bool bmi_160::init() {
    ESP_LOGI(TAG, "Starting BMI160 initialization...");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify chip ID
    uint8_t chip_id;
    if (!i2c_.read_registers(0x69, 0x00, &chip_id, 1)) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return false;
    }
    ESP_LOGI(TAG, "Chip ID: 0x%02x", chip_id);

    // Power up accelerometer
    if (!i2c_.write_register(0x69, 0x7E, 0x11)) {  // Normal power mode
        ESP_LOGE(TAG, "Failed to start accelerometer");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Power up gyroscope
    if (!i2c_.write_register(0x69, 0x7E, 0x15)) {  // Normal power mode
        ESP_LOGE(TAG, "Failed to start gyroscope");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Configure accelerometer (ODR = 100Hz, bandwidth = normal)
    if (!i2c_.write_register(0x69, 0x40, 0x28)) {  // ACC_CONF register
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return false;
    }

    // Configure gyroscope (ODR = 100Hz, bandwidth = normal)
    if (!i2c_.write_register(0x69, 0x42, 0x28)) {  // GYR_CONF register
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return false;
    }

    // Set accelerometer range (±2g = 0x03, ±4g = 0x05, ±8g = 0x08, ±16g = 0x0C)
    if (!i2c_.write_register(0x69, 0x41, 0x03)) {  // ACC_RANGE register
        ESP_LOGE(TAG, "Failed to set accelerometer range");
        return false;
    }
    
    // Set gyroscope range (±125°/s = 0x04, ±250°/s = 0x03, ±500°/s = 0x02, ±1000°/s = 0x01, ±2000°/s = 0x00)
    if (!i2c_.write_register(0x69, 0x43, 0x03)) {  // GYR_RANGE register
        ESP_LOGE(TAG, "Failed to set gyroscope range");
        return false;
    }

    uint8_t acc_conf, acc_range;
    if (i2c_.read_registers(0x69, 0x40, &acc_conf, 1)) {
        ESP_LOGI(TAG, "ACC_CONF: 0x%02x", acc_conf);
    }
    if (i2c_.read_registers(0x69, 0x41, &acc_range, 1)) {
        ESP_LOGI(TAG, "ACC_RANGE: 0x%02x", acc_range);
    }

    uint8_t pmu_status;
    if (i2c_.read_registers(0x69, REG_PMU_STATUS, &pmu_status, 1)) {
        ESP_LOGI(TAG, "PMU_STATUS: 0x%02x", pmu_status);
        // Bits 5:4 should be 0b01 for gyro normal mode
        // Bits 7:6 should be 0b01 for accel normal mode
    }

    ESP_LOGI(TAG, "Full initialization complete");
    return true;
}
    

void bmi_160::check_accel_data() {
    uint8_t data[6];
    if (i2c_.read_registers(MPU_ADDR, REG_DATA_START + 6, data, 6)) {  // Start from accel data
        ESP_LOGI(TAG, "Direct accel bytes: %02x %02x %02x %02x %02x %02x",
            data[0], data[1], data[2], data[3], data[4], data[5]);
    }
}


bmi_160::mpu_reading bmi_160::read() {
    
    mpu_reading reading;
    uint8_t data[12];  // BMI160 uses 12 bytes instead of 14
    reading.timestamp = xTaskGetTickCount();

    if (!i2c_.read_registers(MPU_ADDR, REG_DATA_START, data, 12)) {
        reading.status = ESP_FAIL;
        return reading;
    }

    // check_accel_data();  // Debug: check raw accel data

    // Print raw bytes to debug
    // ESP_LOGI(TAG, "Raw bytes: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
    //     data[0], data[1], data[2], data[3], data[4], data[5],
    //     data[6], data[7], data[8], data[9], data[10], data[11]);

    // Both gyro and accel are little-endian in BMI160
    reading.gyro[0] = (int16_t(data[1] << 8 | data[0]) - offsets_.gx) * GYRO_SCALE;
    reading.gyro[1] = (int16_t(data[3] << 8 | data[2]) - offsets_.gy) * GYRO_SCALE;
    reading.gyro[2] = (int16_t(data[5] << 8 | data[4]) - offsets_.gz) * GYRO_SCALE;

    reading.accel[0] = (int16_t(data[7] << 8 | data[6]) - offsets_.ax) * ACCEL_SCALE;
    reading.accel[1] = (int16_t(data[9] << 8 | data[8]) - offsets_.ay) * ACCEL_SCALE;
    reading.accel[2] = (int16_t(data[11] << 8 | data[10]) - offsets_.az) * ACCEL_SCALE;

    // ESP_LOGI(TAG, "IMU reading: Accel: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f)",
    //     reading.accel[0], reading.accel[1], reading.accel[2],
    //     reading.gyro[0], reading.gyro[1], reading.gyro[2]
    // );

    return reading;
}

esp_err_t bmi_160::calibrate(uint16_t samples) {
    int32_t accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
    int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    uint8_t data[14];

    // Temporarily store current offsets
    int16_t old_accel_x = accel_offset_x_;
    int16_t old_accel_y = accel_offset_y_;
    int16_t old_accel_z = accel_offset_z_;
    int16_t old_gyro_x = gyro_offset_x_;
    int16_t old_gyro_y = gyro_offset_y_;
    int16_t old_gyro_z = gyro_offset_z_;

    // Zero out offsets for calibration
    accel_offset_x_ = accel_offset_y_ = accel_offset_z_ = 0;
    gyro_offset_x_ = gyro_offset_y_ = gyro_offset_z_ = 0;

    // Collect samples
    for (uint16_t i = 0; i < samples; i++) {
        if (!i2c_.read_registers(MPU_ADDR, 0x3B, data, 14)) {
            // Restore old offsets on failure
            accel_offset_x_ = old_accel_x;
            accel_offset_y_ = old_accel_y;
            accel_offset_z_ = old_accel_z;
            gyro_offset_x_ = old_gyro_x;
            gyro_offset_y_ = old_gyro_y;
            gyro_offset_z_ = old_gyro_z;
            return ESP_FAIL;
        }

        // Accumulate raw readings
        accel_x_sum += int16_t(data[0] << 8 | data[1]);
        accel_y_sum += int16_t(data[2] << 8 | data[3]);
        accel_z_sum += int16_t(data[4] << 8 | data[5]);
        gyro_x_sum += int16_t(data[8] << 8 | data[9]);
        gyro_y_sum += int16_t(data[10] << 8 | data[11]);
        gyro_z_sum += int16_t(data[12] << 8 | data[13]);

        vTaskDelay(pdMS_TO_TICKS(2)); // Small delay between readings
    }

    // Calculate average offsets
    accel_offset_x_ = accel_x_sum / samples;
    accel_offset_y_ = accel_y_sum / samples;
    accel_offset_z_ = accel_z_sum / samples;
    gyro_offset_x_ = gyro_x_sum / samples;
    gyro_offset_y_ = gyro_y_sum / samples;
    gyro_offset_z_ = gyro_z_sum / samples;

    // For accelerometer, adjust Z axis to remove gravity (assuming sensor is flat)
    accel_offset_y_ += int16_t(9.81f / ACCEL_SCALE); // Remove 1g from Z axis

    ESP_LOGI(TAG, "Calibration complete. New offsets: Accel: (%.2i, %.2i, %.2i), Gyro: (%.2i, %.2i, %.2i)",
        accel_offset_x_, accel_offset_y_, accel_offset_z_,
        gyro_offset_x_, gyro_offset_y_, gyro_offset_z_
    );
    return ESP_OK;
}