#include "esp_log.h"
#include "sensors/mpu_6050.hpp"


bool mpu_6050::init() {
    // Wake up MPU6050 (clear sleep bit)
    uint8_t data = 0x00;
    if (!i2c_.write_register(MPU_ADDR, 0x6B, data)) {  // PWR_MGMT_1 register
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return false;
    }

    // Configure gyro range (±250°/s)
    data = 0x00;  // 0x00 = 250deg/s
    if (!i2c_.write_register(MPU_ADDR, 0x1B, data)) {  // GYRO_CONFIG register
        ESP_LOGE(TAG, "Failed to configure gyro range");
        return false;
    }

    // Configure accelerometer range (±2g)
    data = 0x00;  // 0x00 = ±2g
    if (!i2c_.write_register(MPU_ADDR, 0x1C, data)) {  // ACCEL_CONFIG register
        ESP_LOGE(TAG, "Failed to configure accelerometer range");
        return false;
    }

    // Optional: Configure digital low pass filter
    data = 0x03;  // Example: bandwidth 44Hz
    if (!i2c_.write_register(MPU_ADDR, 0x1A, data)) {  // CONFIG register
        ESP_LOGE(TAG, "Failed to configure DLPF");
        return false;
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully, returning");
    return true;
}

mpu_6050::mpu_reading mpu_6050::read() {
    mpu_reading reading;
    uint8_t data[14];
    reading.timestamp = xTaskGetTickCount();

    if (!i2c_.read_registers(MPU_ADDR, 0x3B, data, 14)) {  // Read 14 bytes starting from ACCEL_XOUT_H
        reading.status = ESP_FAIL;
        return reading;
    }

    // Combine high and low bytes for each sensor
    reading.accel[0] = (int16_t(data[0] << 8 | data[1]) - offsets_.ax) * ACCEL_SCALE;
    reading.accel[1] = (int16_t(data[2] << 8 | data[3]) - offsets_.ay) * ACCEL_SCALE;
    reading.accel[2] = (int16_t(data[4] << 8 | data[5]) - offsets_.az) * ACCEL_SCALE;
    
    reading.gyro[0] = (int16_t(data[8] << 8 | data[9]) - offsets_.gx) * GYRO_SCALE;
    reading.gyro[1] = (int16_t(data[10] << 8 | data[11]) - offsets_.gy) * GYRO_SCALE;
    reading.gyro[2] = (int16_t(data[12] << 8 | data[13]) - offsets_.gz) * GYRO_SCALE;

    // ESP_LOGI(TAG, "IMU reading: Accel: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f)",
    //     reading.accel[0], reading.accel[1], reading.accel[2],
    //     reading.gyro[0], reading.gyro[1], reading.gyro[2]
    // );
    return reading;
}

esp_err_t mpu_6050::calibrate(uint16_t samples) {
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