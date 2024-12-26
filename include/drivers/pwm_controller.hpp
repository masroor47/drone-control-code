#pragma once

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

class pwm_controller {
public:
    // PWM configuration parameters
    struct config {
        uint32_t frequency;          // PWM frequency in Hz
        uint8_t resolution_bits;     // PWM resolution (1-20 bits)
        ledc_timer_t timer_num;      // Timer number
        ledc_mode_t speed_mode;      // PWM speed mode
    };

    static pwm_controller& getInstance() {
        static pwm_controller instance;
        return instance;
    }

    // Initialize a PWM timer
    bool initTimer(const config& config) {
        ledc_timer_config_t timer_conf = {
            .speed_mode = config.speed_mode,
            .duty_resolution = static_cast<ledc_timer_bit_t>(config.resolution_bits),
            .timer_num = config.timer_num,
            .freq_hz = config.frequency,
            .clk_cfg = LEDC_AUTO_CLK
        };

        esp_err_t err = ledc_timer_config(&timer_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure PWM timer: %s", esp_err_to_name(err));
            return false;
        }
        return true;
    }

private:
    pwm_controller() = default;
    static constexpr const char* TAG = "PWM";
};