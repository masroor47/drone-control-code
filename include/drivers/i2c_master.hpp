#pragma once
#include <cstdint>
#include "esp_log.h"
#include "driver/i2c.h"

class i2c_master {
public:
    static constexpr uint32_t DEFAULT_FREQ = 400000;  // 400 KHz
    static i2c_master& get_instance() {
        static i2c_master instance;
        return instance;
    };
    bool init(gpio_num_t sda, gpio_num_t scl, uint32_t frequency);
    bool read_registers(uint8_t device_addr, uint8_t start_reg, uint8_t* data, size_t len);
    bool write_register(uint8_t device_addr, uint8_t reg, uint8_t data);
    
private:
    static constexpr const char* TAG = "I2C";
    i2c_port_t i2c_port_ = I2C_NUM_0;
    bool initialized_ = false;
    bool pins_configured_ = false;
    i2c_master() = default;
    ~i2c_master() {
        i2c_driver_delete(I2C_NUM_0);
    }
    // Prevent copying
    i2c_master(const i2c_master&) = delete;
    i2c_master& operator=(const i2c_master&) = delete;
};