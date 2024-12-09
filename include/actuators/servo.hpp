#pragma once
#include "driver/gpio.h"

class servo {
private:
    const gpio_num_t pin_;
    static uint32_t servo_angle_to_pulsewidth(float);
public:
    static constexpr uint32_t MIN_PULSE_US = 500;   // Minimum pulse width in microseconds
    static constexpr uint32_t MAX_PULSE_US = 2500;  // Maximum pulse width in microseconds
    static constexpr float MAX_DEGREE = 180.0f;     // Maximum angle in degrees
    
    servo(gpio_num_t pin);
    static void init();
    static void set_control_surfaces(float);
};
