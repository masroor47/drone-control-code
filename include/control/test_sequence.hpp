#pragma once

#include <vector>
#include <memory>
#include <functional>
#include "actuators/servo.hpp"

class test_sequence {
public:
    struct config {
        gpio_num_t gyro_servo1_pin;
        gpio_num_t gyro_servo2_pin;
        ledc_channel_t gyro_channel1;
        ledc_channel_t gyro_channel2;
        ledc_timer_t timer_num;
        ledc_mode_t speed_mode;
    };

    struct test_state {
        uint32_t current_test_idx = 0;
        uint32_t step_counter = 0;
        uint32_t duration_ms = 0;
        bool test_complete = false;
    };

    explicit test_sequence(const config& cfg);
    bool init();
    void update();
    bool is_complete() const { return state_.test_complete; }
    
private:
    // Test definition structure
    struct test_step {
        std::function<void(test_sequence&)> execute;
        uint32_t duration_ms;
        const char* description;
    };

    // Individual test implementations
    void single_axis_sweep(float amplitude, float frequency, int servo_num);
    void dual_axis_circle(float radius, float frequency);
    void phase_lag_test(float amplitude, float phase_degrees);
    void random_motion_test(float max_amplitude);
    void step_response_test(float step_size);

    // Test sequence definition
    void initialize_test_sequence();
    
    config config_;
    test_state state_;
    std::unique_ptr<servo> gyro_servo1_;
    std::unique_ptr<servo> gyro_servo2_;
    std::vector<test_step> test_sequence_;
    
    static constexpr float MAX_ANGLE = 30.0f;  // Maximum rotation angle, not used rn
    static constexpr const char* TAG = "TEST_SEQUENCE";
};