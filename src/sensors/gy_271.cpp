#include "sensors/gy_271.hpp"

bool gy_271::init() {
    // Reset the device
    uint8_t data = 0x80;
    if (!i2c_.write_register(QMC_ADDR, 0x0A, data)) {  // Control Register 2
        ESP_LOGE(TAG, "Failed to reset QMC5883L");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for reset
    
    // Verify chip ID (expected 0xFF for QMC5883L)
    uint8_t chip_id;
    if (!i2c_.read_registers(QMC_ADDR, 0x0D, &chip_id, 1)) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return false;
    }
    ESP_LOGI(TAG, "Chip ID: 0x%02x", chip_id);

    // Configure the sensor
    // 0x1D = Continuous mode (0x01), 2G range (0x00), 200Hz ODR (0x0C), OSR=512 (0x10)
    data = 0x1D;
    if (!i2c_.write_register(QMC_ADDR, 0x09, data)) {  // Control Register 1
        ESP_LOGE(TAG, "Failed to configure QMC5883L");
        return false;
    }
    
    // Set other configurations
    // Control Register 2 (0x0A) = Normal operation (0x00)
    if (!i2c_.write_register(QMC_ADDR, 0x0A, 0x00)) {
        ESP_LOGE(TAG, "Failed to set control register 2");
        return false;
    }
    
    // Set Operation Mode Register (0x0B) = Continuous mode
    if (!i2c_.write_register(QMC_ADDR, 0x0B, 0x01)) {
        ESP_LOGE(TAG, "Failed to set continuous mode");
        return false;
    }

    ESP_LOGI(TAG, "GY271 initialized successfully");
    return true;
}


gy_271::mag_reading gy_271::read() {
    mag_reading reading;
    reading.timestamp = xTaskGetTickCount();

    // Check status register first
    uint8_t status;
    if (!i2c_.read_registers(QMC_ADDR, 0x06, &status, 1)) {
        ESP_LOGE(TAG, "Failed to read status register");
        reading.status = ESP_FAIL;
        return reading;
    }
    
    // Check if data is ready (bit 0 of status register)
    if (!(status & 0x01)) {
        ESP_LOGW(TAG, "Data not ready (status: 0x%02x)", status);
        reading.status = ESP_FAIL;
        return reading;
    }

    uint8_t data[6];
    // Read all magnetometer registers at once
    if (!i2c_.read_registers(QMC_ADDR, 0x00, data, 6)) {
        ESP_LOGE(TAG, "Failed to read status");
        reading.status = ESP_FAIL;
        return reading;
    }

    // Combine bytes and convert to microTesla
    reading.mag[0] = (int16_t(data[0] | data[1] << 8)) * MAG_SCALE;
    reading.mag[1] = (int16_t(data[2] | data[3] << 8)) * MAG_SCALE;
    reading.mag[2] = (int16_t(data[4] | data[5] << 8)) * MAG_SCALE;
    
    // Calculate heading (in radians)
    reading.heading = atan2(reading.mag[1], reading.mag[0]);
    // Correct for when signs are reversed
    if (reading.heading < 0) {
        reading.heading += 2 * M_PI;
    }
    
    reading.status = ESP_OK;

    // ESP_LOGI(TAG, "Mag reading: (%.2f, %.2f, %.2f) uT, Heading: %.2f rad",
    //     reading.mag[0], reading.mag[1], reading.mag[2], reading.heading
    // );
    return reading;
}

