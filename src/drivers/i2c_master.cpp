#include "esp_log.h"
#include "drivers/i2c_master.hpp"

bool i2c_master::init(gpio_num_t sda, gpio_num_t scl, uint32_t frequency) {
        ESP_LOGI(TAG, "Initializing I2C master");
        if (initialized_) {
                ESP_LOGW(TAG, "I2C already initialized");
                return true;
        }

        // First cleanup any existing configuration
        if (pins_configured_) {
            i2c_driver_delete(i2c_port_);
        }

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
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(err));
            return false;
        }
        pins_configured_ = true;

        err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
            return false;
        }
        initialized_ = true;
        return true;
    }

bool i2c_master::read_registers(uint8_t deviceAddr, uint8_t startReg, uint8_t* data, size_t len) {
    if (data == nullptr) {
        ESP_LOGE(TAG, "Invalid null data pointer");
        return false;
    }

    if (len == 0) {
        ESP_LOGE(TAG, "Invalid length of 0");
        return false;
    }

    // ESP_LOGI(TAG, "Reading %d bytes from register 0x%02X on device 0x%02X", len, startReg, deviceAddr);
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        ESP_LOGE(TAG, "Failed to create I2C command link");
        return false;
    }

    esp_err_t ret;
    
    // First write the register address we want to start reading from
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add START condition: %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return false;
    }

    ret = i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write device address (write mode): %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return false;
    }

    ret = i2c_master_write_byte(cmd, startReg, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address: %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return false;
    }
    
    // Repeated start for reading
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add repeated START condition: %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return false;
    }

    ret = i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_READ, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write device address (read mode): %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return false;
    }
    
    if (len > 1) {
        ret = i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read data bytes: %s", esp_err_to_name(ret));
            i2c_cmd_link_delete(cmd);
            return false;
        }
    }

    ret = i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read last data byte: %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return false;
    }
    
    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add STOP condition: %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return false;
    }

    ret = i2c_master_cmd_begin(i2c_port_, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to execute I2C transaction: %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return false;
    }

    i2c_cmd_link_delete(cmd);
    
    // Verify data was actually read
    bool all_zero = true;
    for (size_t i = 0; i < len; i++) {
        if (data[i] != 0) {
            all_zero = false;
            break;
        }
    }
    if (all_zero) {
        ESP_LOGW(TAG, "Warning: All read values are 0, possible communication issue");
    }
    // ESP_LOGI(TAG, "Read data: %02X %02X %02X %02X %02X %02X", data[0], data[1], data[2], data[3], data[4], data[5]);
    
    return true;
}

bool i2c_master::write_register(uint8_t device_addr, uint8_t reg, uint8_t data) {
    if (!initialized_) {
        ESP_LOGE(TAG, "Attempt to write before I2C initialization");
        return false;
    }
    // log what we are writing and to where
    // ESP_LOGI(TAG, "Writing 0x%02X to register 0x%02X on device 0x%02X", data, reg, device_addr);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}