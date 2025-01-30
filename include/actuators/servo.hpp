#pragma once
#include "driver/gpio.h"
#include "driver/ledc.h"

class servo {
public:
    struct config {
        gpio_num_t pin;
        ledc_channel_t channel;
        ledc_timer_t timer_num;
        ledc_mode_t speed_mode;
        float min_pulse_ms;  // Minimum pulse width in milliseconds
        float max_pulse_ms;  // Maximum pulse width in milliseconds
        float min_angle;     // Minimum angle in degrees
        float max_angle;     // Maximum angle in degrees
        float calib_offset;  // Calibration offset in degrees
        float angle_limit;   // Artificial angle limit to prevent vanes from touching
    };

    explicit servo(const config& cfg) 
        : config_(cfg) {}

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
            ESP_LOGE(TAG, "Failed to configure servo channel: %s", esp_err_to_name(err));
            return false;
        }

        // Set to middle position
        set_angle((config_.max_angle + config_.min_angle) / 2);
        return true;
    }

    void set_angle(float angle) {
        // Constrain angle
        // ESP_LOGI(TAG, "Setting angle to %f", angle);
        if (angle < config_.min_angle) angle = config_.min_angle;
        if (angle > config_.max_angle) angle = config_.max_angle;
        if (abs(angle) > config_.angle_limit) angle = config_.angle_limit * (angle < 0 ? -1 : 1);
        
        angle += config_.calib_offset;

        // Convert angle to pulse width in milliseconds
        float pulse_width_ms = config_.min_pulse_ms + 
            (angle - config_.min_angle) * 
            (config_.max_pulse_ms - config_.min_pulse_ms) / 
            (config_.max_angle - config_.min_angle);

        // Convert to duty cycle (assuming 20ms period for standard servos)
        uint32_t duty = (pulse_width_ms / 20.0f) * ((1 << SERVO_RESOLUTION_BITS) - 1);
        // ESP_LOGI(TAG, "Setting duty to %d", (int)duty);
        
        ledc_set_duty(config_.speed_mode, config_.channel, duty);
        ledc_update_duty(config_.speed_mode, config_.channel);
    }

private:
    const config config_;
    static constexpr const char* TAG = "Servo";
    static constexpr uint8_t SERVO_RESOLUTION_BITS = 14;  // 14-bit resolution
};
