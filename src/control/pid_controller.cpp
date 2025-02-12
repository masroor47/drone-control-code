#include <algorithm>
#include "control/pid_controller.hpp"

float pid_controller::update(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    // Calculate error
    const float error = setpoint - measurement;

    // Proportional term
    const float p_term = config_.kp * error;

    // Integral term with anti-windup
    integrator_ += error * dt;
    integrator_ = std::clamp(integrator_, config_.integrator_min, config_.integrator_max);
    const float i_term = config_.ki * integrator_;

    // Derivative term (on measurement to avoid setpoint spikes)
    // Note: we use measurement instead of error to avoid derivative kicks
    float derivative = (measurement - prev_measurement_) / dt;
    
    // Low pass filter for derivative term
    if (config_.derivative_lpf_alpha > 0.0f) {
        filtered_derivative_ = config_.derivative_lpf_alpha * derivative + 
                             (1.0f - config_.derivative_lpf_alpha) * filtered_derivative_;
        derivative = filtered_derivative_;
    }
    
    const float d_term = -config_.kd * derivative;  // Negative because we used measurement

    // Calculate total output
    float output = p_term + i_term + d_term;

    // Clamp output
    output = std::clamp(output, config_.output_min, config_.output_max);

    // Store previous values
    prev_measurement_ = measurement;

    return output;
}

void pid_controller::reset() {
    prev_error_ = 0.0f;
    integrator_ = 0.0f;
    filtered_derivative_ = 0.0f;
    prev_measurement_ = 0.0f;
}