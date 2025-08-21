#include "control/flight_control_system.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_task_wdt.h"

#include <memory>
#include <cstring>  // for memcpy

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


bool flight_control_system::init() {

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
        .angle_limit = 45.0f
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

    pitch_attitude_pid_ = std::make_unique<pid_controller>(pid_configs_.pitch_attitude);
    yaw_attitude_pid_ = std::make_unique<pid_controller>(pid_configs_.yaw_attitude);
    roll_rate_pid_ = std::make_unique<pid_controller>(pid_configs_.roll_rate);
    pitch_rate_pid_ = std::make_unique<pid_controller>(pid_configs_.pitch_rate);
    yaw_rate_pid_ = std::make_unique<pid_controller>(pid_configs_.yaw_rate);
    
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

        float pitch_angle_sp = -mapped_channels.pitch_angle_deg;
        float yaw_angle_sp = mapped_channels.roll_angle_deg;


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

    uint16_t rc_channels[RC_INPUT_MAX_CHANNELS];
    uint16_t num_channels;

    float throttle_percent;
    
    float test_angle = 0;

    TickType_t timestamp;

    float roll_rate_sp = 0;
    float pitch_rate_sp = 0;
    float yaw_rate_sp = 0;

    while (true) {
        if (xSemaphoreTake(system.rc_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(rc_channels, system.rc_data_.data.channels, 
                   sizeof(uint16_t) * 10);
            xSemaphoreGive(system.rc_data_.mutex);
        }
        mapped_channels = rc_mapper::map_channels(rc_channels);
        // if (tick++ % 10 == 0) {
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
        if (xSemaphoreTake(system.rate_setpoint_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {;
            roll_rate_sp = system.rate_setpoint_data_.setpoints.roll_rate;
            pitch_rate_sp = system.rate_setpoint_data_.setpoints.pitch_rate;
            yaw_rate_sp = system.rate_setpoint_data_.setpoints.yaw_rate;
            xSemaphoreGive(system.rate_setpoint_data_.mutex);
        }

        // Get latest gyro data
        mpu_6050::mpu_reading gyro_data;
        // bmi_160::mpu_reading gyro_data;
        xSemaphoreTake(system.imu_data_.mutex, pdMS_TO_TICKS(5));
        gyro_data = system.imu_data_.latest_reading;
        xSemaphoreGive(system.imu_data_.mutex);
        
        // pitch_rate_sp = -mapped_channels.pitch_angle_deg * 0.5f; // Convert to rate
        // yaw_rate_sp = 0.0f;
        // yaw_rate_sp = mapped_channels.roll_angle_deg * 0.5f; // Convert to rate
        roll_rate_sp = mapped_channels.yaw_rate_rad_s;
        // roll_rate_sp = 0.0f;

        // pitch_rate_sp = 0.0f;
        // yaw_rate_sp = 0.0f;

        // precession correction here, not in rate control
        float theta_rad = mapped_channels.wheel * M_PI / 2;

        float pitch_rate_sp_coupled = pitch_rate_sp * cos(theta_rad) + yaw_rate_sp * sin(theta_rad);
        float yaw_rate_sp_coupled = -pitch_rate_sp * sin(theta_rad) + yaw_rate_sp * cos(theta_rad);


        // Run rate PIDs
        float roll_thrust = system.roll_rate_pid_->update(
            roll_rate_sp, gyro_data.gyro[1], 0.005f);

        float pitch_thrust = -system.pitch_rate_pid_->update(
            pitch_rate_sp_coupled, -gyro_data.gyro[0], 0.005f);

        float yaw_thrust = -system.yaw_rate_pid_->update(
            yaw_rate_sp_coupled, -gyro_data.gyro[2], 0.005f);

        // float pitch_thrust = mapped_channels.pitch_angle_deg;
        // float yaw_thrust = -mapped_channels.roll_angle_deg;
        // yaw_thrust = 0;

        throttle_percent = mapped_channels.throttle_percent;
        system.esc_->set_throttle(throttle_percent);

        float K_PRECESSION = 0.0f;
        // float K_PRECESSION = mapped_channels.wheel * 100.0f;
        // const float precession_gain = K_PRECESSION;// * throttle_percent;  // Experimentally determined
        // const float comp_pitch = pitch_thrust*(1-precession_gain) + precession_gain * yaw_thrust;
        // const float comp_yaw = yaw_thrust*(1-precession_gain) - precession_gain * pitch_thrust;
        const float comp_pitch = pitch_thrust + K_PRECESSION * gyro_data.gyro[2]; // perpendicular component for precession
        const float comp_yaw = yaw_thrust + K_PRECESSION * gyro_data.gyro[0]; // perpendicular component for precession

        // Torque compensation
        float K_TORQUE = 0.35f;
        float counter_torque_angle = throttle_percent * K_TORQUE;
        
        const float servo1_cmd = -comp_pitch - comp_yaw + roll_thrust - counter_torque_angle;
        const float servo2_cmd =  comp_pitch - comp_yaw + roll_thrust - counter_torque_angle;
        const float servo3_cmd =  comp_pitch + comp_yaw + roll_thrust - counter_torque_angle;
        const float servo4_cmd = -comp_pitch + comp_yaw + roll_thrust - counter_torque_angle;

        // ESP_LOGI(TAG, "motor: %2f%%, gyro: %.2f, %.2f, %.2f; pitch_th: %.2f, yaw_th: %.2f; roll_th: %.2f,  comp_pitch: %.2f, comp_yaw: %.2f",
        //     throttle_percent, gyro_data.gyro[0], gyro_data.gyro[1], gyro_data.gyro[2], pitch_thrust, yaw_thrust, roll_thrust, comp_pitch, comp_yaw
        // );
        if (tick++ % 8 == 0) {
            ESP_LOGI(TAG, "Coupled Desired Rates: %.2f, %.2f; gyro: %.2f, %.2f, %.2f; pitch_thrust: %.2f, yaw_thrust: %.2f, K_PREC: %.2f",
                pitch_rate_sp_coupled, yaw_rate_sp_coupled,
                gyro_data.gyro[0], gyro_data.gyro[1], gyro_data.gyro[2],
                pitch_thrust, yaw_thrust, K_PRECESSION
            );
            // ESP_LOGI(TAG, "Rate sp: %2.2f, %2.2f, %2.2f; throttle: %2.2f; Servo commands: %2.2f, %2.2f, %2.2f, %2.2f",
            //     roll_rate_sp, pitch_rate_sp, yaw_rate_sp, throttle_percent,
            //         servo1_cmd, servo2_cmd, servo3_cmd, servo4_cmd
            //     );
        }



        system.servos_[0]->set_angle(servo1_cmd);
        system.servos_[1]->set_angle(servo2_cmd);
        system.servos_[2]->set_angle(servo3_cmd);
        system.servos_[3]->set_angle(servo4_cmd);

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

                    // if (++tick % 10 == 0) {
                    //     ESP_LOGI(TAG, "Channels: %d, %d, %d, %d, %d, %d, %d, %d, %2d, %2d",
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