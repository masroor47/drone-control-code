#pragma once
#include <cstdint>
#include <cmath>
#include "esp_err.h"
#include "drivers/i2c_master.hpp"

class gy_271 {
public:
    static constexpr uint8_t QMC_ADDR = 0x0D;  // QMC5883L I2C address
    
    explicit gy_271(i2c_master& i2c) : i2c_(i2c) {};
    bool init();
    struct mag_reading {
        float mag[3];        // Magnetic field in microTesla (uT)
        float heading;       // Heading in radians
        esp_err_t status;
        TickType_t timestamp;
    };
    mag_reading read();
    
private:
    static constexpr char* TAG = "GY271";
    
    // Scale for 2G range (default). Different from HMC5883L!
    static constexpr float MAG_SCALE = 0.0000833f;  // Convert to mT (2G range)
    
    i2c_master& i2c_;
};