#pragma once
#include <cstdint>

class pid_controller {
public:
    struct config {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float output_min = -1.0f;
        float output_max = 1.0f;
        float integrator_min = -1.0f;
        float integrator_max = 1.0f;
        float derivative_lpf_alpha = 0.0f;  // Low-pass filter for D term
    };

    explicit pid_controller(const config& cfg)
        : config_(cfg),
          prev_error_(0.0f),
          integrator_(0.0f),
          filtered_derivative_(0.0f),
          prev_measurement_(0.0f) {}

    float update(float setpoint, float measurement, float dt);
    void reset();
    void update_config(const config& new_cfg) {
        config_ = new_cfg;
        reset();
    }

private:
    config config_;
    float prev_error_;
    float integrator_;
    float filtered_derivative_;
    float prev_measurement_;
};