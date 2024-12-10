#pragma once
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

class esc_controller {
public:
    struct config {
        gpio_num_t pin;
        ledc_channel_t channel;
        ledc_timer_t timer_num;
        ledc_mode_t speed_mode;
        float min_throttle_ms;  // Minimum pulse width (usually 1ms)
        float max_throttle_ms;  // Maximum pulse width (usually 2ms)
    };

    explicit esc_controller(const config& config) 
        : config_(config), initialized_(false) {}

    bool init() {
        ledc_channel_config_t chan_conf = {
            .gpio_num = config_.pin,
            .speed_mode = config_.speed_mode,
            .channel = config_.channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = config_.timer_num,
            .duty = 0,
            .hpoint = 0
        };

        esp_err_t err = ledc_channel_config(&chan_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure ESC channel: %s", esp_err_to_name(err));
            return false;
        }

        // Perform ESC calibration sequence if needed
        if (!calibrate_esc()) {
            return false;
        }

        initialized_ = true;
        return true;
    }

    void setThrottle(float throttle_percent) {
        if (!initialized_) return;

        // Constrain throttle to 0-100%
        if (throttle_percent < 0.0f) throttle_percent = 0.0f;
        if (throttle_percent > 100.0f) throttle_percent = 100.0f;

        // Convert throttle to pulse width
        float pulse_width_ms = config_.min_throttle_ms + 
            (throttle_percent / 100.0f) * 
            (config_.max_throttle_ms - config_.min_throttle_ms);

        // Convert to duty cycle (assuming 20ms period)
        uint32_t duty = (pulse_width_ms / 20.0f) * ((1 << ESC_RESOLUTION_BITS) - 1);
        
        ledc_set_duty(config_.speed_mode, config_.channel, duty);
        ledc_update_duty(config_.speed_mode, config_.channel);
    }

private:
    bool calibrate_esc() {
        // Optional: Implement ESC calibration sequence if needed
        // This depends on your ESC type and requirements
        return true;
    }

    const config config_;
    bool initialized_;
    static constexpr const char* TAG = "ESC";
    static constexpr uint8_t ESC_RESOLUTION_BITS = 14;  // 14-bit resolution
};