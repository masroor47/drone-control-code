#pragma once
#include <cstdint>
#include "driver/i2c.h"

class I2CMaster {
public:
    static constexpr uint32_t DEFAULT_FREQ = 400000;  // 400 KHz
    
    struct Config {
        int sda_pin;
        int scl_pin;
        uint32_t frequency = DEFAULT_FREQ;
        i2c_port_t port = I2C_NUM_0;
    };
    
    explicit I2CMaster(const Config& config);
    
private:
    const Config config_;
};