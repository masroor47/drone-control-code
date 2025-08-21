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
            .kp = 12.0f,
            .ki = 0.00f,
            .kd = 0.00f,
            .output_min = -M_PI * 1.5,  // -180 deg/s
            .output_max = M_PI * 1.5,   // 180 deg/s
            .integrator_min = -M_PI * 1.5,
            .integrator_max = M_PI * 1.5,
            .derivative_lpf_alpha = 0.1f
        };
        configs.yaw_attitude = {
            .kp = 12.0f,
            .ki = 0.00f,
            .kd = 0.00f,
            .output_min = -M_PI * 1.5,  // -180 deg/s
            .output_max = M_PI * 1.5,   // 180 deg/s
            .integrator_min = -M_PI * 1.5,
            .integrator_max = M_PI * 1.5,
            .derivative_lpf_alpha = 0.1f
        };


        // Rate PIDs
        configs.pitch_rate = {
            .kp = 2.0f,
            .ki = 0.0f,
            .kd = 1.2f,
            .output_min = -45.0f,  // vane deflection angle
            .output_max = 45.0f,
            .integrator_min = -0.2f,
            .integrator_max = 0.2f,
            .derivative_lpf_alpha = 0.2f
        };
        configs.yaw_rate = {
            .kp = 2.0f,
            .ki = 0.0f,
            .kd = 1.2f,
            .output_min = -45.0f,  // vane deflection angle
            .output_max = 45.0f,
            .integrator_min = -0.2f,
            .integrator_max = 0.2f,
            .derivative_lpf_alpha = 0.2f
        };

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