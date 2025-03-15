#pragma once
#include <stdint.h>
#include <algorithm>

class rc_mapper {
public:
    // Configuration constants
    static constexpr float MAX_ANGLE_DEG = 45.0f;  // Maximum roll/pitch angle in degrees
    static constexpr float MAX_YAW_RATE_RAD_S = 3.14159f;  // Maximum yaw rate in rad/s
    static constexpr float MAX_THROTTLE_PERCENT = 50.0f;  // Maximum allowed throttle
    
    // RC input ranges (adjust based on your transmitter)
    static constexpr uint16_t RC_MIN = 1000;
    static constexpr uint16_t RC_MAX = 2000;
    static constexpr uint16_t RC_MID = 1500;
    
    // Armed channel thresholds
    static constexpr uint16_t ARMED_THRESHOLD = 1800;  // Above this is considered armed

    struct mapped_channels {
        float roll_angle_deg;
        float pitch_angle_deg;
        float yaw_rate_rad_s;
        float throttle_percent;
        bool armed;
        float wheel;
    };

    static mapped_channels map_channels(const uint16_t* rc_channels) {
        mapped_channels mapped;
        
        // Map roll and pitch from RC input to angle range
        mapped.roll_angle_deg = map_range_symmetric(
            rc_channels[0], RC_MIN, RC_MAX, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
        mapped.pitch_angle_deg = map_range_symmetric(
            rc_channels[1], RC_MIN, RC_MAX, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
            
        // Map throttle to percentage, limiting maximum. Value is high when not receiving signal
        // TODO: fix, make it armed dependent
        if (rc_channels[2] > RC_MAX + 100) {
            mapped.throttle_percent = 0;
        } else {
            mapped.throttle_percent = map_range(
                rc_channels[2], RC_MIN, RC_MAX, 0.0f, MAX_THROTTLE_PERCENT);
        }

        // Map yaw to rate range
        mapped.yaw_rate_rad_s = map_range_symmetric(
            rc_channels[3], RC_MIN, RC_MAX, -MAX_YAW_RATE_RAD_S, MAX_YAW_RATE_RAD_S);
            
        // Map armed channel to boolean
        mapped.armed = rc_channels[4] > ARMED_THRESHOLD;

        mapped.wheel = map_range(rc_channels[9], RC_MIN, RC_MAX, 0.0f, 1.0f);
        
        return mapped;
    }

private:
    static float map_range(uint16_t x, uint16_t in_min, uint16_t in_max, 
                            float out_min, float out_max) {
        float normalized = static_cast<float>(x - in_min) / (in_max - in_min);
        normalized = std::clamp(normalized, 0.0f, 1.0f);
        return out_min + normalized * (out_max - out_min);
    }
    
    static float map_range_symmetric(uint16_t x, uint16_t in_min, uint16_t in_max,
                                    float out_min, float out_max) {
        float normalized = static_cast<float>(x - RC_MID) / (RC_MAX - RC_MID);
        normalized = std::clamp(normalized, -1.0f, 1.0f);
        return normalized * std::max(std::abs(out_min), std::abs(out_max));
    }
};