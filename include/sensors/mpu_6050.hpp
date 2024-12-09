#pragma once
#include <cstdint>
#include <stdio.h>
#include <cmath>
#include "esp_err.h"


class mpu_6050 {
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x68;
    
    mpu_6050(int sda_pin, int scl_pin);
    bool init();
    
private:
    static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;  // Convert to m/s² (±2g mode)
    static constexpr float GYRO_SCALE = M_PI / (180.0f * 131.0f);  // Convert to rad/s
    
    int16_t accel_offset_x_, accel_offset_y_, accel_offset_z_;
    int16_t gyro_offset_x_, gyro_offset_y_, gyro_offset_z_;
    
    const int sda_pin_; 
    const int scl_pin_;
};