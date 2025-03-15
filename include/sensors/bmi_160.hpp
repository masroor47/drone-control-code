#pragma once
#include <cstdint>
#include <cmath>
#include "esp_err.h"
#include "drivers/i2c_master.hpp"


class bmi_160 {
public:
    static constexpr uint8_t MPU_ADDR = 0x69;

    struct offsets {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
    };
    static constexpr offsets DEFAULT_OFFSETS = {
        0, 0, 0,   // accel
        0, 0, 0     // gyro
    };
    explicit bmi_160(i2c_master& i2c) : 
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
    static constexpr const char* TAG = "BMI160";

    static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;  // Convert to m/s² (±2g mode)
    // static constexpr float GYRO_SCALE = M_PI / (180.0f * 131.0f);  // Convert to rad/s
    static constexpr float GYRO_SCALE = M_PI / (180.0f * 16.4f);  // For ±2000°/s mode

    static constexpr uint8_t REG_CMD = 0x7E;
    static constexpr uint8_t REG_PMU_STATUS = 0x03;
    static constexpr uint8_t REG_ACC_CONF = 0x40;
    static constexpr uint8_t REG_GYR_CONF = 0x42;
    static constexpr uint8_t REG_ACC_RANGE = 0x41;
    static constexpr uint8_t REG_GYR_RANGE = 0x43;
    static constexpr uint8_t REG_DATA_START = 0x12;  // Instead of 0x3B for MPU6050
    
    int16_t accel_offset_x_, accel_offset_y_, accel_offset_z_;
    int16_t gyro_offset_x_, gyro_offset_y_, gyro_offset_z_;
    offsets offsets_;
    i2c_master& i2c_;

    void check_accel_data();
};