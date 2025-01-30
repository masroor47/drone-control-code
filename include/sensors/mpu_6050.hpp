#pragma once
#include <cstdint>
#include <cmath>
#include "esp_err.h"
#include "drivers/i2c_master.hpp"


class mpu_6050 {
public:
    static constexpr uint8_t MPU_ADDR = 0x68;

    struct offsets {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
    };
    static constexpr offsets DEFAULT_OFFSETS = {
        563, -362, -1195,   // accel
        -984, 140, 114     // gyro
    };
    explicit mpu_6050(i2c_master& i2c) : 
        offsets_(DEFAULT_OFFSETS), 
        i2c_(i2c) {};

    bool init();
    struct mpu_reading {
        float accel[3];
        float gyro[3];
        esp_err_t status;
        TickType_t timestamp;
    };
    mpu_reading read();
    esp_err_t calibrate(uint16_t samples = 5000);
    
private:
    static constexpr const char* TAG = "MPU6050";

    static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;  // Convert to m/s² (±2g mode)
    static constexpr float GYRO_SCALE = M_PI / (180.0f * 131.0f);  // Convert to rad/s
    
    int16_t accel_offset_x_, accel_offset_y_, accel_offset_z_;
    int16_t gyro_offset_x_, gyro_offset_y_, gyro_offset_z_;
    offsets offsets_;
    i2c_master& i2c_;
};