#pragma once
#include <cmath>
#include "control/pid_controller.hpp"
#include "control/pid_configs.hpp"
#include "control/flight_control_system.hpp"

struct flight_control_config {
    static constexpr pid_configs default_pid_configs() {
        pid_configs configs;
        
        // Attitude PIDs
        configs.pitch_attitude = {
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .output_min = -M_PI/4,  // rad/s = 45 deg/s
            .output_max = M_PI/4,   // rad/s = 45 deg/s
            .integrator_min = -10.0f,
            .integrator_max = 10.0f,
            .derivative_lpf_alpha = 0.1f
        };
        configs.yaw_attitude = {
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .output_min = -M_PI/4,  // rad/s = 45 deg/s
            .output_max = M_PI/4,   // rad/s = 45 deg/s
            .integrator_min = -10.0f,
            .integrator_max = 10.0f,
            .derivative_lpf_alpha = 0.1f
        };


        // Rate PIDs
        configs.pitch_rate = {
            .kp = 7.0f,
            .ki = 0.5f,
            .kd = 0.0f,
            .output_min = -45.0f,  // vane deflection angle
            .output_max = 45.0f,
            .integrator_min = -0.2f,
            .integrator_max = 0.2f,
            .derivative_lpf_alpha = 0.2f
        };
        configs.yaw_rate = configs.pitch_rate;

        configs.roll_rate = {
            .kp = 10.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .output_min = -45.0f,
            .output_max = 45.0f,
            .integrator_min = -0.2f,
            .integrator_max = 0.2f,
            .derivative_lpf_alpha = 0.2f
        };
        
        return configs;
    }
};