#pragma once
#include <cstdint>
#include "driver/i2c.h"

class i2c_master {
public:
    static constexpr uint32_t DEFAULT_FREQ = 400000;  // 400 KHz
    static i2c_master& get_instance() {
        static i2c_master instance;
        return instance;
    };
    bool init(gpio_num_t sda, gpio_num_t scl, uint32_t frequency) {
        i2c_config_t conf = {
            mode: I2C_MODE_MASTER,
            sda_io_num: sda,
            scl_io_num: scl,
            sda_pullup_en: GPIO_PULLUP_ENABLE,
            scl_pullup_en: GPIO_PULLUP_ENABLE,
            master: {
                clk_speed: frequency
            }
        };
        
        esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
        if (err != ESP_OK) return false;

        err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
        return err == ESP_OK;
    }
    
    bool read_registers(uint8_t device_addr, uint8_t start_reg, uint8_t* data, size_t len);
    bool write_register(uint8_t device_addr, uint8_t reg, uint8_t data);
    
private:
    i2c_port_t i2c_port_ = I2C_NUM_0;
    i2c_master() = default;
    ~i2c_master() {
        i2c_driver_delete(I2C_NUM_0);
    }
};