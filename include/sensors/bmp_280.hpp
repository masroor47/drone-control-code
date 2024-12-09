
#pragma once
#include <cstdint>
#include "esp_err.h"


class bmp_280 {
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x77;
    
    struct Registers {
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

    bmp_280(int sda_pin, int scl_pin);
    bool init();
    static esp_err_t read_data(float*, float*);

private:
    const int sda_pin_;
    const int scl_pin_;
};