#include "esp_log.h"
#include "sensors/bmp_280.hpp"

bool bmp_280::init() {
    // Reset device
    if (!i2c_.write_register(BMP_ADDR, 0xE0, 0xB6)) {
        ESP_LOGE(TAG, "Failed to reset BMP280");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Check ID
    uint8_t id;
    if (!i2c_.read_registers(BMP_ADDR, 0xD0, &id, 1) || id != 0x58) {
        ESP_LOGE(TAG, "Invalid BMP280 ID");
        return false;
    }

    // Read calibration (simplified)
    uint8_t calib_data[24];
    if (!i2c_.read_registers(BMP_ADDR, 0x88, calib_data, 24)) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return false;
    }

    parse_calibration_data(calib_data);

    // Configure sensor
    if (!i2c_.write_register(BMP_ADDR, 0xF4, 0x57)) {
        ESP_LOGE(TAG, "Failed to configure BMP280");
        return false;
    }
    ESP_LOGI(TAG, "BMP280 initialized successfully");

    return true;
}

bmp_280::reading bmp_280::read() {
    reading reading;
    uint8_t data[6];
    reading.timestamp = xTaskGetTickCount();

    if (!i2c_.read_registers(BMP_ADDR, registers::TEMP_MSB, data, 6)) {
        reading.status = ESP_FAIL;
        return reading;
    }

    int32_t adc_T = (int32_t(data[3]) << 12) | (int32_t(data[4]) << 4) | (int32_t(data[5]) >> 4);
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib_.dig_T1 << 1))) * ((int32_t)calib_.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib_.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_.dig_T1))) >> 12) * ((int32_t)calib_.dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    reading.temperature = (t_fine * 5 + 128) >> 8;

    int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
    var1 = (((int32_t)t_fine) >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib_.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)calib_.dig_P4) << 16);
    var1 = (((calib_.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + 
            ((((int32_t)calib_.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)calib_.dig_P1)) >> 15);
    
    if (var1 == 0) { // Avoid division by zero
        reading.status = ESP_FAIL;
        return reading;
    }  
    
    int32_t p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t)var1);
    } else {
        p = (p / (uint32_t)var1) * 2;
    }

    var1 = (((int32_t)calib_.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)calib_.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + calib_.dig_P7) >> 4));
    
    reading.pressure = p / 100.0f;  // Convert to hPa
    reading.status = ESP_OK;
    return reading;
}

void bmp_280::parse_calibration_data(uint8_t* data) {
    calib_.dig_T1 = (uint16_t)(data[1] << 8) | data[0];
    calib_.dig_T2 = (int16_t)(data[3] << 8) | data[2];
    calib_.dig_T3 = (int16_t)(data[5] << 8) | data[4];
    calib_.dig_P1 = (uint16_t)(data[7] << 8) | data[6];
    calib_.dig_P2 = (int16_t)(data[9] << 8) | data[8];
    calib_.dig_P3 = (int16_t)(data[11] << 8) | data[10];
    calib_.dig_P4 = (int16_t)(data[13] << 8) | data[12];
    calib_.dig_P5 = (int16_t)(data[15] << 8) | data[14];
    calib_.dig_P6 = (int16_t)(data[17] << 8) | data[16];
    calib_.dig_P7 = (int16_t)(data[19] << 8) | data[18];
    calib_.dig_P8 = (int16_t)(data[21] << 8) | data[20];
    calib_.dig_P9 = (int16_t)(data[23] << 8) | data[22];
}