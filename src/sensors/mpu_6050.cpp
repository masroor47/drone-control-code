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

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
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
    reading.accel[0] = (int16_t(data[0] << 8 | data[1])) * ACCEL_SCALE;
    reading.accel[1] = (int16_t(data[2] << 8 | data[3])) * ACCEL_SCALE;
    reading.accel[2] = (int16_t(data[4] << 8 | data[5])) * ACCEL_SCALE;
    reading.gyro[0] = (int16_t(data[8] << 8 | data[9])) * GYRO_SCALE;
    reading.gyro[1] = (int16_t(data[10] << 8 | data[11])) * GYRO_SCALE;
    reading.gyro[2] = (int16_t(data[12] << 8 | data[13])) * GYRO_SCALE;
    reading.status = ESP_OK;

    return reading;
}