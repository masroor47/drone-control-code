#include "control/flight_control_system.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_task_wdt.h"

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_http_server.h"

#include "cJSON.h"

#include <memory>
#include <cstring>  // for memcpy
#include "freertos/queue.h"

#include "esp_log.h"
#include "control/pid_controller.hpp"
#include "control/rc_mapper.hpp"
#include "drivers/i2c_master.hpp"
#include "sensors/mpu_6050.hpp"
#include "sensors/bmp_280.hpp"
#include "drivers/pwm_controller.hpp"
#include "actuators/servo.hpp"
#include "actuators/esc_controller.hpp"
#include "utils/thread_safe_queue.hpp"

#include "radio/crsf.h"


const char* NVS_NAMESPACE = "pid";

void flight_control_system::scan_i2c() {
    auto &i2c_ = i2c_master::get_instance();
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 0x00; addr < 0x7F; addr++) {
        uint8_t data;
        if (i2c_.read_registers(addr, 0x00, &data, 1)) {
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
        }
    }
};

void load_pid_from_nvs(nvs_handle_t handle, flight_control_system::pid_entry& entry) {
    uint32_t temp_u32;
    float temp_float;
    const char *TAG = "load from nvs";

    if (nvs_get_u32(handle, entry.kp_key, &temp_u32) == ESP_OK) {
        memcpy(&temp_float, &temp_u32, sizeof(float));
        entry.cfg->kp = temp_float;
    } else {
        ESP_LOGE(TAG, "Failed to read %s from NVS", entry.kp_key);
    }
    if (nvs_get_u32(handle, entry.ki_key, &temp_u32) == ESP_OK) {
        memcpy(&temp_float, &temp_u32, sizeof(float));
        entry.cfg->ki = temp_float;
    } else {
        ESP_LOGE(TAG, "Failed to read %s from NVS", entry.ki_key);
    }
    if (nvs_get_u32(handle, entry.kd_key, &temp_u32) == ESP_OK) {
        memcpy(&temp_float, &temp_u32, sizeof(float));
        entry.cfg->kd = temp_float;
    } else {
        ESP_LOGE(TAG, "Failed to read %s from NVS", entry.kd_key);
    }
}

void save_pid_to_nvs(nvs_handle_t handle, const flight_control_system::pid_entry& entry) {
    uint32_t temp_u32;
    memcpy(&temp_u32, &entry.cfg->kp, sizeof(float));
    nvs_set_u32(handle, entry.kp_key, temp_u32);

    memcpy(&temp_u32, &entry.cfg->ki, sizeof(float));
    nvs_set_u32(handle, entry.ki_key, temp_u32);

    memcpy(&temp_u32, &entry.cfg->kd, sizeof(float));
    nvs_set_u32(handle, entry.kd_key, temp_u32);
}


bool flight_control_system::init() {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash: %s\nRetrying...", esp_err_to_name(ret));
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash: %s", esp_err_to_name(ret));
        return false;
    }

    wifi_init_ap();
    ws_server_ = start_webserver();
    if (ws_server_ == nullptr) {
        ESP_LOGE(TAG, "Failed to start WS server");
        return false;
    }

    if (!i2c_master::get_instance().init(config_.i2c_sda, config_.i2c_scl, config_.i2c_freq)) {
        ESP_LOGE(TAG, "Failed to initialize I2C master");
        return false;
    }
    // scan_i2c(); // Uncomment to scan I2C bus to find connected devices

    imu_ = std::make_unique<mpu_6050>(i2c_master::get_instance());
    // imu_ = std::make_unique<bmi_160>(i2c_master::get_instance());
    barometer_ = std::make_unique<bmp_280>(i2c_master::get_instance());
    mag_ = std::make_unique<gy_271>(i2c_master::get_instance());

    if (!imu_->init()) {
        ESP_LOGE(TAG, "Failed to initialize IMU");
        return false;
    }
    ESP_LOGI(TAG, "IMU initialized successfully");

    if (!barometer_->init()) {
        ESP_LOGE(TAG, "Failed to initialize BMP280");
        return false;
    }
    ESP_LOGI(TAG, "BMP280 initialized successfully");

    // if (!mag_->init()) {
    //     ESP_LOGE(TAG, "Failed to initialize GY271");
    //     return false;
    // }
    // ESP_LOGI(TAG, "GY271 initialized successfully");

    // Initialize PWM timers first
    pwm_controller::config servo_timer_config {
        .frequency = 50,              // 50Hz for servos
        .resolution_bits = 14,        // 14-bit resolution
        .timer_num = LEDC_TIMER_0,
        .speed_mode = LEDC_LOW_SPEED_MODE
    };

    pwm_controller::config esc_timer_config {
        .frequency = 50,              // 50Hz for ESC
        .resolution_bits = 14,        // 14-bit resolution
        .timer_num = LEDC_TIMER_1,    // Different timer for ESC
        .speed_mode = LEDC_LOW_SPEED_MODE
    };

    if (!pwm_controller::getInstance().initTimer(servo_timer_config) ||
        !pwm_controller::getInstance().initTimer(esc_timer_config)) {
        return false;
    }
    ESP_LOGI(TAG, "PWM timers initialized successfully");

    // Initialize servos
    servo::config servo_config {
        .pin = this->config_.servo1_pin,
        .channel = LEDC_CHANNEL_0,
        .timer_num = LEDC_TIMER_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .min_pulse_ms = 0.5f,    // 1ms
        .max_pulse_ms = 2.5f,    // 2ms
        .min_angle = -90.0f,
        .max_angle = 90.0f,
        .calib_offset = 0.0f,
        .angle_limit = 40.0f
    };

    const std::array<gpio_num_t, 4> servo_pins = {
        config_.servo1_pin, config_.servo2_pin, 
        config_.servo3_pin, config_.servo4_pin
    };

    const std::array<ledc_channel_t, 4> servo_channels = {
        LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3
    };

    const std::array<float, 4> calib_offset = {
        -12.0f, 6.0f, -5.0f, 7.0f
    };

    for (size_t i = 0; i < servos_.size(); ++i) {
        servo_config.pin = servo_pins[i];
        servo_config.channel = servo_channels[i];
        servo_config.calib_offset = calib_offset[i];

        servos_[i] = std::make_unique<servo>(servo_config);
        if (!servos_[i]->init()) {
            ESP_LOGE(TAG, "Failed to initialize servo %d", i);
            return false;
        }
    }
    ESP_LOGI(TAG, "Servos initialized successfully");

    esc_controller::config esc_config {
        .pin = GPIO_NUM_13,
        .channel = LEDC_CHANNEL_4,
        .timer_num = LEDC_TIMER_1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .min_throttle_ms = 1.0f,  // 1ms
        .max_throttle_ms = 2.0f   // 2ms
    };

    esc_ = std::make_unique<esc_controller>(esc_config);
    if (!esc_->init()) {
        return false;
    }
    ESP_LOGI(TAG, "ESC initialized successfully");

    pid_configs_ = flight_control_config::default_pid_configs();
    nvs_handle_t nvs_handle;
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        for (auto& entry : pid_entries_) {
            load_pid_from_nvs(nvs_handle, entry);
        }
    } else {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", NVS_NAMESPACE, esp_err_to_name(ret));
    }
    nvs_close(nvs_handle);

    pitch_attitude_pid_ = std::make_unique<pid_controller>(pid_configs_.pitch_attitude);
    yaw_attitude_pid_ = std::make_unique<pid_controller>(pid_configs_.yaw_attitude);
    roll_rate_pid_ = std::make_unique<pid_controller>(pid_configs_.roll_rate);
    pitch_rate_pid_ = std::make_unique<pid_controller>(pid_configs_.pitch_rate);
    yaw_rate_pid_ = std::make_unique<pid_controller>(pid_configs_.yaw_rate);

    ESP_LOGI(TAG, "PID controllers initialized successfully");

    pid_config_controller_map_[&pid_configs_.pitch_attitude] = pitch_attitude_pid_.get();
    pid_config_controller_map_[&pid_configs_.yaw_attitude] = yaw_attitude_pid_.get();
    pid_config_controller_map_[&pid_configs_.roll_rate] = roll_rate_pid_.get();
    pid_config_controller_map_[&pid_configs_.pitch_rate] = pitch_rate_pid_.get();
    pid_config_controller_map_[&pid_configs_.yaw_rate] = yaw_rate_pid_.get();

    return true;
}

void flight_control_system::imu_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(5);  // 200Hz
    int log_counter = 0;

    while (true) {
        auto imu_reading = system.imu_->read();
        if (xSemaphoreTake(system.imu_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            system.imu_data_.latest_reading = imu_reading;
            xSemaphoreGive(system.imu_data_.mutex);
        }
        // if (log_counter++ % 10 == 0) {
        //     ESP_LOGI(TAG, "IMU reading: Accel: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f)",
        //         imu_reading.accel[0], imu_reading.accel[1], imu_reading.accel[2],
        //         imu_reading.gyro[0], imu_reading.gyro[1], imu_reading.gyro[2]
        //     );
        // }

        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

void flight_control_system::sensor_fusion_task(void* param) {
    ESP_ERROR_CHECK(esp_task_wdt_add(xTaskGetCurrentTaskHandle()));

    auto& system = *static_cast<flight_control_system*>(param);
    const float RAD_TO_DEG = 180.0f / M_PI;
    const float TICK_TO_SEC = 1.0f / configTICK_RATE_HZ;

    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for good IMU readings

    TickType_t last_wake_time;
    last_wake_time = xTaskGetTickCount();

    const TickType_t task_period = pdMS_TO_TICKS(10);  // 100Hz

    static int log_counter = 0;
    while (true) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        
        mpu_6050::mpu_reading imu_reading;
        // bmi_160::mpu_reading imu_reading;
        if (xSemaphoreTake(system.imu_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            imu_reading = system.imu_data_.latest_reading;
            xSemaphoreGive(system.imu_data_.mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        TickType_t current_ticks = xTaskGetTickCount();

        if (system.filter_state_.last_update == 0) {
            system.filter_state_.last_update = current_ticks;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        float dt = (float)(current_ticks - system.filter_state_.last_update) * TICK_TO_SEC;

        float accel_norm = sqrt(
            imu_reading.accel[0] * imu_reading.accel[0] +
            imu_reading.accel[1] * imu_reading.accel[1] +
            imu_reading.accel[2] * imu_reading.accel[2]
        );
    
        if (accel_norm > 0.1f) {
            // Calculate pitch and roll from accelerometer
            // Remember: Y is down in your setup
            float ax = imu_reading.accel[0] / accel_norm;
            float ay = imu_reading.accel[1] / accel_norm;
            float az = imu_reading.accel[2] / accel_norm;
            
            // Get angles from accelerometer
            float accel_pitch = atan2(-az, sqrt(ay*ay + ax*ax)) * RAD_TO_DEG;
            float accel_yaw = atan2(ax, -ay) * RAD_TO_DEG;
            
            // Integrate gyro rates
            float gyro_pitch = system.filter_state_.pitch - 
                imu_reading.gyro[0] * system.filter_params_.gyro_scale * dt * RAD_TO_DEG;
            float gyro_yaw = system.filter_state_.yaw - 
                imu_reading.gyro[2] * system.filter_params_.gyro_scale * dt * RAD_TO_DEG;
            

            // Complementary filter
            // Higher alpha = trust accelerometer more, but more noise
            // Lower alpha = smoother but might drift
            float alpha = system.filter_params_.alpha;
            
            system.filter_state_.pitch = (1.0f - alpha) * gyro_pitch + alpha * accel_pitch;
            system.filter_state_.yaw = (1.0f - alpha) * gyro_yaw + alpha * accel_yaw;
            
            // if (log_counter++ % 10 == 0) {
                // log raw imu readings
                // ESP_LOGI(TAG, "IMU reading: Accel: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f)",
                //     imu_reading.accel[0], imu_reading.accel[1], imu_reading.accel[2],
                //     imu_reading.gyro[0], imu_reading.gyro[1], imu_reading.gyro[2]
                // );
                // // ESP_LOGI(TAG, "Apitch: %.2f, Ayaw: %.2f; Gpitch: %.2f, Gyaw: %.2f", 
                // //     accel_pitch, accel_yaw, gyro_pitch, gyro_yaw);
                // ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
                //     system.filter_state_.roll, system.filter_state_.pitch, system.filter_state_.yaw
                // );
            // }
        }
        system.filter_state_.last_update = current_ticks;

        if (xSemaphoreTake(system.attitude_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            system.attitude_data_.latest_estimate.pitch = system.filter_state_.pitch;
            system.attitude_data_.latest_estimate.yaw = system.filter_state_.yaw;
            xSemaphoreGive(system.attitude_data_.mutex);
        }

        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

void flight_control_system::barometer_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);

    while (true) {
        auto baro_reading = system.barometer_->read();
        if (xSemaphoreTake(system.barometer_data_.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            system.barometer_data_.latest_reading = baro_reading;
            xSemaphoreGive(system.barometer_data_.mutex);
        }
        // ESP_LOGI(TAG, "Barometer reading: Pressure: %.2f Pa, Temperature: %.2f C",
        //     baro_reading.pressure, baro_reading.temperature
        // );

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void flight_control_system::mag_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);

    while (true) {
        auto mag_reading = system.mag_->read();
        // ESP_LOGI(TAG, "Magnetometer reading: (%.2f, %.2f, %.2f)",
        //     mag_reading.mag[0], mag_reading.mag[1], mag_reading.mag[2]
        // );

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void flight_control_system::control_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);
    flight_control_system::attitude_estimate attitude;

    int delayed_i, i = 0;
    float throttle_percent;
    float counter_angle;
    float max_throttle_percent = 25.0f;
    float rpm_counter_coeff = 0.4f;

    while (true) {
        if (xSemaphoreTake(system.attitude_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            attitude = system.attitude_data_.latest_estimate;
            xSemaphoreGive(system.attitude_data_.mutex);
            // ESP_LOGI(TAG, "Attitude: Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
            //     attitude.roll, attitude.pitch, attitude.yaw
            // );
        }
        // points to the ground, example servo mixing
        // system.servos_[0]->set_angle(attitude.pitch + attitude.yaw);
        // system.servos_[1]->set_angle(-attitude.pitch + attitude.yaw);
        // system.servos_[2]->set_angle(-attitude.pitch - attitude.yaw);
        // system.servos_[3]->set_angle(attitude.pitch - attitude.yaw);


        // counters the torque
        delayed_i = std::max(0, ++i-500); // delay the throttle up by 10 seconds
        throttle_percent = -pow(cos(delayed_i * 0.01f), 2) * max_throttle_percent + max_throttle_percent;
        system.esc_->set_throttle(throttle_percent);
        
        counter_angle = throttle_percent * rpm_counter_coeff;
        ESP_LOGI(TAG, "throttle: %.2f%%, compen angle: %.2f", 
            throttle_percent, -counter_angle);

        system.servos_[0]->set_angle(-counter_angle);
        system.servos_[1]->set_angle(-counter_angle);
        system.servos_[2]->set_angle(-counter_angle);
        system.servos_[3]->set_angle(-counter_angle);

        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}

void flight_control_system::attitude_control_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz

    uint16_t rc_channels[RC_INPUT_MAX_CHANNELS];
    rc_mapper::mapped_channels mapped_channels;

    int tick = 0;
    
    while (true) {
        if (xSemaphoreTake(system.rc_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(rc_channels, system.rc_data_.data.channels, 
                   sizeof(uint16_t) * 10);
            xSemaphoreGive(system.rc_data_.mutex);
        }
        mapped_channels = rc_mapper::map_channels(rc_channels);

        float pitch_angle_sp = 0; //-mapped_channels.pitch_angle_deg;
        float yaw_angle_sp = 0; //mapped_channels.roll_angle_deg;


        // Get latest attitude estimate
        attitude_estimate current_attitude;
        if (xSemaphoreTake(system.attitude_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {;
            current_attitude = system.attitude_data_.latest_estimate;
            xSemaphoreGive(system.attitude_data_.mutex);
        }
        
        // Run attitude PIDs
        float desired_pitch_rate = system.pitch_attitude_pid_->update(
            pitch_angle_sp, current_attitude.pitch, 0.01f);
        float desired_yaw_rate = system.yaw_attitude_pid_->update(
            yaw_angle_sp, current_attitude.yaw, 0.01f);

        // if (tick++ % 4 == 0) {
        //     ESP_LOGI(TAG, "Attitude: Desired: %.2f, %.2f; Current: %.2f, Yaw: %.2f; Desired rates: %.2f, %.2f",
        //         pitch_angle_sp, yaw_angle_sp,
        //         current_attitude.pitch, current_attitude.yaw, 
        //         desired_pitch_rate, desired_yaw_rate
        //     );
        // }
        
        // Update rate setpoints
        if (xSemaphoreTake(system.rate_setpoint_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {;
            system.rate_setpoint_data_.setpoints.roll_rate = 0.0;
            system.rate_setpoint_data_.setpoints.pitch_rate = desired_pitch_rate;
            system.rate_setpoint_data_.setpoints.yaw_rate = desired_yaw_rate;
            xSemaphoreGive(system.rate_setpoint_data_.mutex);
        }
        
        vTaskDelayUntil(&last_wake_time, period);
    }
}

// temporary function to simulate throttle ramp up and down
float calculate_throttle(int elapsed_ms, float max_throttle = 25.0f) {
   const int initial_delay = 8000;
   const int ramp_up = 4000;
   const int sustain = 30000;
   const int ramp_down = 2000;
   
   if (elapsed_ms < initial_delay) return 0.0f;
   if (elapsed_ms < initial_delay + ramp_up) {
       float x = (float)(elapsed_ms - initial_delay) / ramp_up;
       return max_throttle * (1.0f / (1.0f + exp(-10.0f * (x - 0.5f))));
   }
   if (elapsed_ms < initial_delay + ramp_up + sustain) return max_throttle;
   if (elapsed_ms < initial_delay + ramp_up + sustain + ramp_down) {
       float x = (float)(elapsed_ms - (initial_delay + ramp_up + sustain)) / ramp_down;
       return max_throttle * (1.0f - x);
   }
   return 0.0f;
}

void flight_control_system::rate_control_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);
    rc_mapper::mapped_channels mapped_channels;

    TickType_t last_wake_time = xTaskGetTickCount();
    int tick = 0;
    const int period_ms = 20;
    const TickType_t period = pdMS_TO_TICKS(5);  // 200Hz

    TickType_t timestamp;

    uint16_t rc_channels[RC_INPUT_MAX_CHANNELS];
    uint16_t num_channels = 0;

    float throttle_percent;

    static float ramp_cmd = 0.0f;
    static bool left_prev = false;
    static const float RAMP_TARGET_PERCENT = 25.0f;
    static const float TAU_UP_S = 0.5f;
    static const float TAU_DOWN_S = 0.4f;
    const float DT_S = 0.005f;

    // For thrust kick
    static bool sd_prev = false;
    static bool kick_active = false;
    static float kick_t = 0.0f;
    static float KICK_DURATION_S = 0.50f;
    static const float KICK_MAX = -0.5f;

    float kick_val = 0.0f;

    float roll_rate_sp = 0;
    float pitch_rate_sp = 0;
    float yaw_rate_sp = 0;

    while (true) {
        tick++;
        if (xSemaphoreTake(system.rc_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(rc_channels, system.rc_data_.data.channels, 
                   sizeof(uint16_t) * 10);
            num_channels = system.rc_data_.data.num_channels;
            TickType_t last_update = system.rc_data_.data.last_update;
            xSemaphoreGive(system.rc_data_.mutex);
            
            TickType_t current_tick = xTaskGetTickCount();
            mapped_channels = rc_mapper::map_channels(rc_channels, num_channels, last_update, current_tick);
        }

        // Rising edge detection: trigger one-shot
        if (mapped_channels.SD && !sd_prev && !kick_active) {
            kick_active = true;
            kick_t = 0.0f;
        }
        sd_prev = mapped_channels.SD;
        if (kick_active) {
            kick_t += DT_S;
            kick_val = KICK_MAX;
            if (kick_t >= KICK_DURATION_S) {
                kick_active = false;
                kick_t = 0.0f;
                kick_val = 0.0f;
            }
        }

        // if (tick % 10 == 0) {
        //     ESP_LOGI(TAG, "RC channels: %d, %d, %d, %d, %d; Mapped RC: roll=%.1f° pitch=%.1f° throttle=%.1f%% yaw=%.1f rad/s armed=%d wheel=%.1f",
        //         rc_channels[0], rc_channels[1], rc_channels[2], rc_channels[3], rc_channels[4],
        //         mapped_channels.roll_angle_deg,
        //         mapped_channels.pitch_angle_deg,
        //         mapped_channels.throttle_percent,
        //         mapped_channels.yaw_rate_rad_s,
        //         mapped_channels.armed, 
        //         mapped_channels.wheel
        //     );
        // }

        // Get latest rate setpoints
        // if (xSemaphoreTake(system.rate_setpoint_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {;
        //     roll_rate_sp = system.rate_setpoint_data_.setpoints.roll_rate; // don't need until we implement global heading lock with compass
        //     pitch_rate_sp = system.rate_setpoint_data_.setpoints.pitch_rate;
        //     yaw_rate_sp = system.rate_setpoint_data_.setpoints.yaw_rate;
        //     xSemaphoreGive(system.rate_setpoint_data_.mutex);
        // }

        // Get latest gyro data
        mpu_6050::mpu_reading gyro_data;
        // bmi_160::mpu_reading gyro_data;
        xSemaphoreTake(system.imu_data_.mutex, pdMS_TO_TICKS(5));
        gyro_data = system.imu_data_.latest_reading;
        xSemaphoreGive(system.imu_data_.mutex);
        
        roll_rate_sp = mapped_channels.yaw_rate_rad_s;

        // Run rate PIDs
        // float roll_thrust = system.roll_rate_pid_->update(
        //     roll_rate_sp, gyro_data.gyro[1], 0.005f);

        pitch_rate_sp = kick_val;

        float pitch_thrust = system.pitch_rate_pid_->update(
            pitch_rate_sp, -gyro_data.gyro[0], 0.005f);

        float yaw_thrust = system.yaw_rate_pid_->update(
            yaw_rate_sp, -gyro_data.gyro[2], 0.005f);

        float roll_thrust = 0.0f;
        // float pitch_thrust = kick_val;
        // float yaw_thrust = 0.0f;


        const float target = mapped_channels.armed ? RAMP_TARGET_PERCENT : 0.0f;
        const bool going_up = (target > ramp_cmd);
        const float tau = going_up ? TAU_UP_S : TAU_DOWN_S;
        const float alpha = DT_S / (tau + DT_S);  // stable discretization
        ramp_cmd += alpha * (target - ramp_cmd);

        // throttle_percent = mapped_channels.throttle_percent;
        throttle_percent = std::clamp(ramp_cmd, 0.0f, rc_mapper::MAX_THROTTLE_PERCENT);
        system.esc_->set_throttle(throttle_percent);

        const float FF_GAIN_PITCH = 25.0f;
        const float FF_GAIN_YAW = 15.0f;

        float comp_pitch = pitch_thrust - FF_GAIN_PITCH * gyro_data.gyro[2];
        float comp_yaw = yaw_thrust + FF_GAIN_YAW * gyro_data.gyro[0];

        // Torque compensation
        float K_TORQUE = 0.35f;
        float counter_torque_angle = throttle_percent * K_TORQUE;
        
        const float servo1_cmd =  comp_pitch + comp_yaw + roll_thrust - counter_torque_angle;
        const float servo2_cmd = -comp_pitch + comp_yaw + roll_thrust - counter_torque_angle;
        const float servo3_cmd = -comp_pitch - comp_yaw + roll_thrust - counter_torque_angle;
        const float servo4_cmd =  comp_pitch - comp_yaw + roll_thrust - counter_torque_angle;

        // ESP_LOGI(TAG, "motor: %2f%%, gyro: %.2f, %.2f, %.2f; pitch_th: %.2f, yaw_th: %.2f; roll_th: %.2f,  comp_pitch: %.2f, comp_yaw: %.2f",
        //     throttle_percent, gyro_data.gyro[0], gyro_data.gyro[1], gyro_data.gyro[2], pitch_thrust, yaw_thrust, roll_thrust, comp_pitch, comp_yaw
        // );
        // if (tick % 8 == 0) {
        //     ESP_LOGI(TAG, "Coupled Desired Rates: %.2f, %.2f; gyro: %.2f, %.2f, %.2f; pitch_thrust: %.2f, yaw_thrust: %.2f, K_PREC: %.2f",
        //         pitch_rate_sp_coupled, yaw_rate_sp_coupled,
        //         gyro_data.gyro[0], gyro_data.gyro[1], gyro_data.gyro[2],
        //         pitch_thrust, yaw_thrust, K_PRECESSION
        //     );
            // ESP_LOGI(TAG, "Rate sp: %2.2f, %2.2f, %2.2f; throttle: %2.2f; Servo commands: %2.2f, %2.2f, %2.2f, %2.2f",
            //     roll_rate_sp, pitch_rate_sp, yaw_rate_sp, throttle_percent,
            //         servo1_cmd, servo2_cmd, servo3_cmd, servo4_cmd
            //     );
        // }

        system.servos_[0]->set_angle(servo1_cmd);
        system.servos_[1]->set_angle(servo2_cmd);
        system.servos_[2]->set_angle(servo3_cmd);
        system.servos_[3]->set_angle(servo4_cmd);

        rate_setpoints rate_sp = {
            .roll_rate = roll_rate_sp,
            .pitch_rate = pitch_rate_sp,
            .yaw_rate = yaw_rate_sp
        };
        thrust_setpoints thrust_sp = {
            .pitch_thrust = comp_pitch,
            .roll_thrust = roll_thrust,
            .yaw_thrust = comp_yaw
        };

        servo_commands cmds = {
            .servo1 = servo1_cmd,
            .servo2 = servo2_cmd,
            .servo3 = servo3_cmd,
            .servo4 = servo4_cmd
        };

        if (tick % 2 == 0) 
            system.update_telemetry_snapshot(gyro_data, rc_channels, rate_sp, thrust_sp, cmds, throttle_percent);

        vTaskDelayUntil(&last_wake_time, period);
    }
}

void flight_control_system::rc_receiver_task(void* param) {
    auto* fcs = static_cast<flight_control_system*>(param);
    const size_t rx_buffer_size = 256;
    uint8_t rx_buffer[rx_buffer_size];
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = (int)fcs->config_.rc_receiver.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // ESP_LOGI(TAG, "Configuring UART%d", (int)fcs->config_.rc_receiver.uart_num);
    
    ESP_ERROR_CHECK(uart_driver_install(fcs->config_.rc_receiver.uart_num, rx_buffer_size * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(fcs->config_.rc_receiver.uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(fcs->config_.rc_receiver.uart_num, 
                                fcs->config_.rc_receiver.tx_pin,
                                fcs->config_.rc_receiver.rx_pin,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE));

    int tick = 0;
    while (true) {
        int length = uart_read_bytes(fcs->config_.rc_receiver.uart_num, 
                                   rx_buffer,
                                   CRSF_BUFFER_SIZE,
                                   pdMS_TO_TICKS(10));
        
        if (length > 0) {
            uint16_t temp_channels[RC_INPUT_MAX_CHANNELS];
            uint16_t num_channels;
            
            if (crsf_parse(rx_buffer, length, temp_channels, &num_channels, RC_INPUT_MAX_CHANNELS) == 0) {
                if (xSemaphoreTake(fcs->rc_data_.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    memcpy(fcs->rc_data_.data.channels, temp_channels, sizeof(uint16_t) * num_channels);
                    fcs->rc_data_.data.num_channels = num_channels;
                    fcs->rc_data_.data.last_update = xTaskGetTickCount();
                    xSemaphoreGive(fcs->rc_data_.mutex);

                    // if (++tick % 5 == 0) {
                    //     ESP_LOGI(TAG, "num_channels: %d, last_update: %d, Channels: %d, %d, %d, %d, %d, %d, %d, %d, %2d, %2d",
                    //         num_channels, (int)fcs->rc_data_.data.last_update,
                    //         temp_channels[0], temp_channels[1], temp_channels[2], temp_channels[3],
                    //         temp_channels[4], temp_channels[5], temp_channels[6], temp_channels[7],
                    //         temp_channels[8], temp_channels[9]
                    //     );
                    // }
                }
            }

        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void flight_control_system::update_telemetry_snapshot(mpu_6050::mpu_reading imu_reading,
                                                      uint16_t *rc_channels,
                                                      rate_setpoints rate_sp,
                                                      thrust_setpoints thrust_sp,
                                                      servo_commands cmds,
                                                      float throttle_percent) {
    telemetry_snapshot snap;
    snap.timestamp = xTaskGetTickCount();

    // Lock and copy from protected data (add error checks)
    xSemaphoreTake(attitude_data_.mutex, portMAX_DELAY);
    snap.attitude = attitude_data_.latest_estimate;
    xSemaphoreGive(attitude_data_.mutex);

    snap.imu = imu_reading;

    // Not using for now
    // xSemaphoreTake(barometer_data_.mutex, portMAX_DELAY);
    // snap.baro = barometer_data_.latest_reading;
    // xSemaphoreGive(barometer_data_.mutex);

    memcpy(snap.rc_channels, rc_channels, sizeof(uint16_t) * 10);

    snap.rate_sp = rate_sp;

    snap.thrust_sp = thrust_sp;

    snap.servo_cmds = cmds;

    snap.throttle_percent = throttle_percent;

    xSemaphoreTake(telemetry_data_.mutex, portMAX_DELAY);
    telemetry_data_.buffer.push_back(snap);
    if (telemetry_data_.buffer.size() > telemetry_data_.max_buffer_size) {
        telemetry_data_.buffer.erase(telemetry_data_.buffer.begin());  // FIFO
        // ESP_LOGI(TAG, "Telemetry buffer full, dropping oldest snapshot");
    }
    xSemaphoreGive(telemetry_data_.mutex);
}

auto safe_add = [](cJSON* arr, double v) {
    const char *TAG = "safe_add";
    cJSON* num = cJSON_CreateNumber(v);
    if (!num) {
        ESP_LOGE(TAG, "Failed to alloc number");
        return false;
    }
    cJSON_AddItemToArray(arr, num);
    return true;
};

void flight_control_system::telemetry_send_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);
    static const char *TAG = "ws_send_telemetry";
    TickType_t period = pdMS_TO_TICKS(10); // 100Hz

    // latch to only print once
    bool no_sockfd_print = false;
    while (true) {
        if (system.ws_socketfd_ == -1) {
            if (!no_sockfd_print) {
                ESP_LOGE(TAG, "Sending telemetry, but sockfd == -1");
                no_sockfd_print = true;
            }
            vTaskDelay(period);
            continue;   
        }

        std::vector<telemetry_snapshot> buffer_copy;
        xSemaphoreTake(system.telemetry_data_.mutex, portMAX_DELAY);
        buffer_copy = system.telemetry_data_.buffer;
        system.telemetry_data_.buffer.clear();
        xSemaphoreGive(system.telemetry_data_.mutex);

        if (buffer_copy.empty()) {
            vTaskDelay(period);
            continue;
        }
        
        cJSON *json = cJSON_CreateObject();
        if (json == NULL) {
            ESP_LOGE(TAG, "Failed to create JSON object");
            vTaskDelay(period);
            continue;
        }
        cJSON_AddStringToObject(json, "type", "telemetry");

        cJSON* data = cJSON_AddArrayToObject(json, "data");
        if (data == NULL) {
            ESP_LOGE(TAG, "Failed to create telemetry array");
            cJSON_Delete(json);
            vTaskDelay(period);
            continue;
        }
        for (const auto& snap : buffer_copy) {
            cJSON* item = cJSON_CreateArray();
            if (item == NULL) continue;

            safe_add(item, snap.timestamp);
            safe_add(item, snap.attitude.roll);
            safe_add(item, snap.attitude.pitch);
            safe_add(item, snap.attitude.yaw);
            safe_add(item, snap.imu.accel[0]);
            safe_add(item, snap.imu.accel[1]);
            safe_add(item, snap.imu.accel[2]);
            safe_add(item, -snap.imu.gyro[0]);
            safe_add(item, snap.imu.gyro[1]);
            safe_add(item, -snap.imu.gyro[2]);
            // safe_add(item, snap.baro.pressure);
            // safe_add(item, snap.baro.temperature);
            safe_add(item, snap.rate_sp.roll_rate);
            safe_add(item, snap.rate_sp.pitch_rate);
            safe_add(item, snap.rate_sp.yaw_rate);
            safe_add(item, snap.thrust_sp.pitch_thrust);
            safe_add(item, snap.thrust_sp.roll_thrust);
            safe_add(item, snap.thrust_sp.yaw_thrust);
            safe_add(item, snap.servo_cmds.servo1);
            safe_add(item, snap.servo_cmds.servo2);
            safe_add(item, snap.servo_cmds.servo3);
            safe_add(item, snap.servo_cmds.servo4);
            safe_add(item, snap.throttle_percent);

            cJSON_AddItemToArray(data, item);
        
        }
        // ESP_LOGI(TAG, "Free heap after JSON object: %" PRIu32, esp_get_free_heap_size());
        // ESP_LOGI(TAG, "Largest free block: %lu", (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        // ESP_LOGI(TAG, "Size of JSON object: %lu", (unsigned long)cJSON_GetArraySize(data));
        char *msg = cJSON_PrintUnformatted(json);

        cJSON_Delete(json);

        if (msg == NULL) {
            ESP_LOGE(TAG, "Failed to create JSON string");
            vTaskDelay(period);
            continue;
        }

        httpd_ws_frame_t ws_pkt = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)msg,
            .len = strlen(msg)
        };
        httpd_ws_send_frame_async(system.ws_server_, system.ws_socketfd_, &ws_pkt);
        free(msg);

        vTaskDelay(period);
    }
}

void flight_control_system::wifi_init_ap() {
    static const char *TAG = "wifi_init_ap";
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32-AP",
            .password = "12345678",
            .ssid_len = strlen("ESP32-AP"),
            .channel = 1,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .ssid_hidden = 0,
            .max_connection = 4,
            .beacon_interval = 100
        },
    };
    if (strlen((char*)wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "WiFi AP started. SSID: %s", wifi_config.ap.ssid);
}

esp_err_t flight_control_system::ws_handler(httpd_req_t *req) {
    auto& system = *static_cast<flight_control_system*>(req->user_ctx);
    static const char *TAG = "ws_handler";

    if (req->method == HTTP_GET) {
        system.ws_socketfd_ = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "Handshake done, the new connection was opened, socket fd: %d", system.ws_socketfd_);

        cJSON* root = cJSON_CreateObject();
        if (root == NULL) {
            ESP_LOGE(TAG, "Failed to create JSON object");
            return ESP_FAIL;
        }
        cJSON_AddStringToObject(root, "type", "pid_config");
        cJSON* data = cJSON_AddObjectToObject(root, "data");
        for (auto& entry : system.pid_entries_) {
            cJSON* pid_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(pid_obj, "kp", entry.cfg->kp);
            cJSON_AddNumberToObject(pid_obj, "ki", entry.cfg->ki);
            cJSON_AddNumberToObject(pid_obj, "kd", entry.cfg->kd);
            cJSON_AddItemToObject(data, entry.name, pid_obj);
        }
        char* msg = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);

        if (msg == NULL) {
            ESP_LOGE(TAG, "Failed to create JSON string");
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Sending PID configs to client");

        httpd_ws_frame_t ws_frame = { 
            .type = HTTPD_WS_TYPE_TEXT, 
            .payload = (uint8_t*)msg, 
            .len = strlen(msg)};

        httpd_ws_send_frame_async(req->handle, system.ws_socketfd_, &ws_frame);
        free(msg);

        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t buf[128] = {0};
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = buf;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(buf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
        return ret;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        ESP_LOGI(TAG, "Received WebSocket text frame");

        cJSON* root = cJSON_Parse((char*)ws_pkt.payload);
        if (root == NULL) {
            ESP_LOGE(TAG, "Failed to parse JSON");
            return ESP_FAIL;
        }

        std::vector<const char*> updated_entries;
        nvs_handle_t nvs_handle;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
            for (auto& entry : system.pid_entries_) {
                cJSON* pid_obj = cJSON_GetObjectItem(root, entry.name);
                if (!pid_obj) {
                    continue;
                }
                cJSON* kp = cJSON_GetObjectItem(pid_obj, "kp");
                cJSON* ki = cJSON_GetObjectItem(pid_obj, "ki");
                cJSON* kd = cJSON_GetObjectItem(pid_obj, "kd");
                if (!kp || !ki || !kd) {
                    ESP_LOGE(TAG, "Invalid PID parameters for %s", entry.name);
                    continue;
                }
                entry.cfg->kp = kp->valuedouble;
                entry.cfg->ki = ki->valuedouble;
                entry.cfg->kd = kd->valuedouble;

                system.pid_config_controller_map_[entry.cfg]->update_config(*entry.cfg);
                ESP_LOGI(TAG, "Updated PID %s: kp=%.4f, ki=%.4f, kd=%.4f",
                    entry.name, entry.cfg->kp, entry.cfg->ki, entry.cfg->kd);

                save_pid_to_nvs(nvs_handle, entry);
                ESP_LOGI(TAG, "Saved to new PID to NVS");

                updated_entries.push_back(entry.name);

            }
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
        } else {
            ESP_LOGE(TAG, "Failed to open NVS, did not update any PID parameters");
        }

        cJSON_Delete(root);

        cJSON* ack_root = cJSON_CreateObject();
        cJSON_AddStringToObject(ack_root, "type", "pid_ack");
        cJSON* ack_data = cJSON_CreateObject();
        cJSON_AddItemToObject(ack_root, "data", ack_data);
        
        for (auto& name : updated_entries) {
            cJSON_AddBoolToObject(ack_data, name, true);
        }
        
        char* ack_str = cJSON_PrintUnformatted(ack_root);
        cJSON_Delete(ack_root);
        ESP_LOGI(TAG, "Free heap after ack_str: %" PRIu32, esp_get_free_heap_size());

        if (ack_str == NULL) {
            ESP_LOGE(TAG, "Failed to create ack_str");
        } else {
            httpd_ws_frame_t ws_frame = {
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t*)ack_str,
                .len = strlen(ack_str)
            };
            httpd_ws_send_frame_async(req->handle, system.ws_socketfd_, &ws_frame);
            free(ack_str);
            ESP_LOGI(TAG, "Free heap after deleting and freeing: %" PRIu32, esp_get_free_heap_size());
        }
    }
    return ESP_OK;
}

httpd_handle_t flight_control_system::start_webserver() {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&server, &config) == ESP_OK) {
        // Register WebSocket
        httpd_uri_t ws = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = ws_handler,
            .user_ctx = this,
            .is_websocket = true
        };
        httpd_register_uri_handler(server, &ws);
        ESP_LOGI(TAG, "Web server started");
    }
    return server;
}

bool flight_control_system::start() {

    BaseType_t ret;

    ret = xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        4096,
        this,
        configMAX_PRIORITIES - 2,
        &imu_task_handle_,
        1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return false;
    }

    ret = xTaskCreatePinnedToCore(
        sensor_fusion_task,
        "sensor_fusion_task",
        4096,
        this,
        configMAX_PRIORITIES - 3,
        &sensor_fusion_task_handle_,
        0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor fusion task");
        return false;
    }

    ret = xTaskCreatePinnedToCore(
        barometer_task,
        "barometer_task",
        4096,
        this,
        configMAX_PRIORITIES - 5,
        &barometer_task_handle_,
        1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create barometer task");
        return false;
    }

    // ret = xTaskCreatePinnedToCore(
    //     mag_task,
    //     "mag_task",
    //     4096,
    //     this,
    //     configMAX_PRIORITIES - 6,
    //     &mag_task_handle_,
    //     1
    // );
    // if (ret != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create magnetometer task");
    //     return false;
    // }


    // ret = xTaskCreatePinnedToCore(
    //     control_task,
    //     "control_task",
    //     4096,
    //     this,
    //     configMAX_PRIORITIES - 4,
    //     &control_task_handle_,
    //     0
    // );
    // if (ret != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create control task");
    //     return false;
    // }

    ret = xTaskCreatePinnedToCore(
        attitude_control_task,
        "attitude_control_task",
        4096,
        this,
        configMAX_PRIORITIES - 4,
        &attitude_control_task_handle_,
        0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create attitude control task");
        return false;
    }

    ret = xTaskCreatePinnedToCore(
        rate_control_task,
        "rate_control_task",
        4096,
        this,
        configMAX_PRIORITIES - 4,
        &rate_control_task_handle_,
        0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create rate control task");
        return false;
    }

    ret = xTaskCreatePinnedToCore(
        telemetry_send_task,
        "telemetry_send_task",
        8192,
        this,
        configMAX_PRIORITIES - 6,
        &telemetry_send_task_handle_,
        1
    );

    ret = xTaskCreatePinnedToCore(
        rc_receiver_task,
        "rc_receiver_task",
        4096,
        this,
        configMAX_PRIORITIES - 3,
        &rc_receiver_task_handle_,
        0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RC receiver task");
        return false;
    }

    return true;
}