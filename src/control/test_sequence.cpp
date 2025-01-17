#include "control/test_sequence.hpp"
#include "esp_log.h"
#include <cmath>

test_sequence::test_sequence(const config& cfg) : config_(cfg) {}

bool test_sequence::init() {
    // Initialize gyro servos
    servo::config servo_config{
        .pin = config_.gyro_servo1_pin,
        .channel = config_.gyro_channel1,
        .timer_num = config_.timer_num,
        .speed_mode = config_.speed_mode,
        .min_pulse_ms = 0.5f,
        .max_pulse_ms = 2.5f,
        .min_angle = -135,
        .max_angle = 135,
        .calib_offset = 0.0f
    };

    gyro_servo1_ = std::make_unique<servo>(servo_config);
    
    servo_config.pin = config_.gyro_servo2_pin;
    servo_config.channel = config_.gyro_channel2;
    gyro_servo2_ = std::make_unique<servo>(servo_config);

    if (!gyro_servo1_->init() || !gyro_servo2_->init()) {
        ESP_LOGE(TAG, "Failed to initialize gyro servos");
        return false;
    }

    initialize_test_sequence();
    return true;
}

void test_sequence::update() {
    if (state_.test_complete) return;
    
    auto& current_test = test_sequence_[state_.current_test_idx];
    
    // Execute current test step
    current_test.execute(*this);
    state_.duration_ms += 20;  // Assuming 20ms update rate
    
    // Check if current test is complete
    if (state_.duration_ms >= current_test.duration_ms) {
        ESP_LOGI(TAG, "Completed test: %s", current_test.description);
        state_.current_test_idx++;
        state_.duration_ms = 0;
        
        if (state_.current_test_idx >= test_sequence_.size()) {
            state_.test_complete = true;
            ESP_LOGI(TAG, "All tests completed");
        }
    }
}

void test_sequence::initialize_test_sequence() {
    test_sequence_ = {
        // Single axis tests
        {
            [this](test_sequence& t) { single_axis_sweep(30.0f, 0.1f, 0); },
            10000,
            "Single axis sweep - X axis - Slow"
        },
        {
            [this](test_sequence& t) { single_axis_sweep(30.0f, 0.2f, 0); },
            5000,
            "Single axis sweep - X axis - Fast"
        },

        {
            [this](test_sequence& t) { single_axis_sweep(30.0f, 0.1f, 1); },
            10000,
            "Single axis sweep - Y axis - Slow"
        },
        {
            [this](test_sequence& t) { single_axis_sweep(30.0f, 0.2f, 1); },
            5000,
            "Single axis sweep - Y axis - Fast"
        },
        
        // Dual axis tests
        {
            [this](test_sequence& t) { dual_axis_circle(30.0f, 0.1f); },
            10000,
            "Dual axis circle - slow"
        },
        {
            [this](test_sequence& t) { dual_axis_circle(30.0f, 0.2f); },
            5000,
            "Dual axis circle - fast"
        },
        
        // Phase lag tests
        // {
        //     [this](test_sequence& t) { phase_lag_test(20.0f, .0f); },
        //     8000,
        //     "Phase lag test - 90 degrees"
        // },
        
        // Random motion
        {
            [this](test_sequence& t) { random_motion_test(30.0f); },
            15000,
            "Random motion test"
        },
        
        // Step response
        {
            [this](test_sequence& t) { step_response_test(20.0f); },
            3000,
            "Step response test"
        }
    };
}

void test_sequence::single_axis_sweep(float amplitude, float frequency, int servo_num) {
    float time_s = state_.duration_ms / 1000.0f;
    float angle = amplitude * std::sin(2.0f * M_PI * frequency * time_s);
    ESP_LOGE(TAG, "single axis Setting servo1 to %f; servo2 to 0", angle);
    if (servo_num == 1) {
        gyro_servo1_->set_angle(angle);
        gyro_servo2_->set_angle(0.0f);
    } else {
        gyro_servo1_->set_angle(0.0f);
        gyro_servo2_->set_angle(angle);
    }
}

void test_sequence::dual_axis_circle(float radius, float frequency) {
    float time_s = state_.duration_ms / 1000.0f;
    float angle_x = radius * std::cos(2.0f * M_PI * frequency * time_s);
    float angle_y = radius * std::sin(2.0f * M_PI * frequency * time_s);
    ESP_LOGE(TAG, "dual axis Setting servo1 to %f; servo2 to %f", angle_x, angle_y);
    gyro_servo1_->set_angle(angle_x);
    gyro_servo2_->set_angle(angle_y);
}

// void test_sequence::phase_lag_test(float amplitude, float phase_degrees) {
//     float time_s = state_.duration_ms / 1000.0f;
//     float phase_rad = phase_degrees * M_PI / 180.0f;
//     float angle1 = amplitude * std::sin(2.0f * M_PI * 0.5f * time_s);
//     float angle2 = amplitude * std::sin(2.0f * M_PI * 0.5f * time_s + phase_rad);
//     gyro_servo1_->set_angle(angle1);
//     gyro_servo2_->set_angle(angle2);
// }

void test_sequence::random_motion_test(float max_amplitude) {
    // Pseudo-random motion using simple harmonic combinations
    float time_s = state_.duration_ms / 1000.0f;
    float angle1 = max_amplitude * (
        0.5f * std::sin(2.0f * M_PI * 0.3f * time_s) +
        0.3f * std::sin(2.0f * M_PI * 0.7f * time_s) +
        0.2f * std::sin(2.0f * M_PI * 1.1f * time_s)
    );
    float angle2 = max_amplitude * (
        0.5f * std::sin(2.0f * M_PI * 0.4f * time_s) +
        0.3f * std::sin(2.0f * M_PI * 0.8f * time_s) +
        0.2f * std::sin(2.0f * M_PI * 1.3f * time_s)
    );
    gyro_servo1_->set_angle(angle1);
    gyro_servo2_->set_angle(angle2);
}

void test_sequence::step_response_test(float step_size) {
    uint32_t step_time_ms = 1000;  // 1 second per step
    uint32_t current_step = state_.duration_ms / step_time_ms;
    
    float angle = (current_step % 2 == 0) ? step_size : -step_size;
    gyro_servo1_->set_angle(angle);
    gyro_servo2_->set_angle(-angle);
}