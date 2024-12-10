
#include "drivers/i2c_master.hpp"

    bool i2c_master::read_registers(uint8_t deviceAddr, uint8_t startReg, uint8_t* data, size_t len) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        
        // First write the register address we want to start reading from
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, startReg, true);
        
        // Then read the specified number of registers
        i2c_master_start(cmd);  // Repeated start
        i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_READ, true);
        
        if (len > 1) {
            // Read all bytes except the last with ACK
            i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
        }
        // Read last byte with NACK
        i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
        
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        return ret == ESP_OK;
    }

bool i2c_master::write_register(uint8_t device_addr, uint8_t reg, uint8_t data) {
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