
#pragma once
#include <cstdint>
#include "esp_err.h"
#include "drivers/i2c_master.hpp"


class bmp_280 {
public:
    static constexpr uint8_t BMP_ADDR = 0x77;
    
    struct registers {
        static constexpr uint8_t TEMP_XLSB = 0xFC;
        static constexpr uint8_t TEMP_LSB = 0xFB;
        static constexpr uint8_t TEMP_MSB = 0xFA;
        static constexpr uint8_t PRESS_XLSB = 0xF9;
        static constexpr uint8_t PRESS_LSB = 0xF8;
        static constexpr uint8_t PRESS_MSB = 0xF7;
        static constexpr uint8_t CONFIG = 0xF5;
        static constexpr uint8_t CTRL_MEAS = 0xF4;
        static constexpr uint8_t STATUS = 0xF3;
        static constexpr uint8_t CTRL_HUM = 0xF2;
        static constexpr uint8_t RESET = 0xE0;
        static constexpr uint8_t ID = 0xD0;
    };

    struct reading {
        float temperature;
        float pressure;
        esp_err_t status;
        TickType_t timestamp;
    };

    bmp_280(i2c_master& i2c) : i2c_(i2c) {};
    bool init();
    void parse_calibration_data(uint8_t* data);
    reading read();

private:
    static constexpr char* TAG = "BMP280";
    i2c_master& i2c_;
    struct calib_param {
        uint16_t dig_T1;
        int16_t  dig_T2;
        int16_t  dig_T3;
        uint16_t dig_P1;
        int16_t  dig_P2;
        int16_t  dig_P3;
        int16_t  dig_P4;
        int16_t  dig_P5;
        int16_t  dig_P6;
        int16_t  dig_P7;
        int16_t  dig_P8;
        int16_t  dig_P9;
        int32_t  t_fine;
    };
    calib_param calib_;

};