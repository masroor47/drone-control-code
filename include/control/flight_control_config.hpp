#include "control/flight_control_system.hpp"
#include "control/pid_controller.hpp"

// flight_control_config.hpp
struct flight_control_config {
    static constexpr flight_control_system::pid_configs default_pid_configs() {
        flight_control_system::pid_configs configs;
        
        // Attitude PIDs
        configs.pitch_attitude = {
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .output_min = -45.0f,  // deg/s
            .output_max = 45.0f,   // deg/s
            .integrator_min = -10.0f,
            .integrator_max = 10.0f,
            .derivative_lpf_alpha = 0.1f
        };
        configs.pitch_attitude = configs.yaw_attitude;  // Usually same as roll


        // Rate PIDs
        configs.pitch_rate = {
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .output_min = -1.0f,  // normalized vane deflection
            .output_max = 1.0f,
            .integrator_min = -0.2f,
            .integrator_max = 0.2f,
            .derivative_lpf_alpha = 0.2f
        };
        configs.yaw_rate = configs.pitch_rate;

        configs.roll_rate = {
            .kp = 0.0f,  // Usually different from roll/pitch
            .ki = 0.0f,
            .kd = 0.0f,
            .output_min = -1.0f,
            .output_max = 1.0f,
            .integrator_min = -0.2f,
            .integrator_max = 0.2f,
            .derivative_lpf_alpha = 0.2f
        };
        
        return configs;
    }
};