#pragma once
#include "control/pid_controller.hpp"

struct pid_configs {
    pid_controller::config pitch_attitude;
    pid_controller::config yaw_attitude;
    pid_controller::config roll_rate;
    pid_controller::config pitch_rate;
    pid_controller::config yaw_rate;
};