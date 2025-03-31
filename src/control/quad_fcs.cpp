#include "control/quad_fcs.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_task_wdt.h"

#include <memory>
#include <algorithm>
#include <cstring>  // for memcpy

#include "esp_log.h"
#include "control/pid_controller.hpp"
#include "control/rc_mapper.hpp"
#include "drivers/i2c_master.hpp"
#include "sensors/bmi_160.hpp"
#include "sensors/bmp_280.hpp"
#include "drivers/pwm_controller.hpp"
#include "actuators/servo.hpp"
#include "actuators/esc_controller.hpp"
#include "utils/thread_safe_queue.hpp"

#include "radio/crsf.h"




void quad_fcs::scan_i2c() {
    auto &i2c_ = i2c_master::get_instance();
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 0x00; addr < 0x7F; addr++) {
        uint8_t data;
        if (i2c_.read_registers(addr, 0x00, &data, 1)) {
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
        }
    }
};


bool quad_fcs::init() {

    if (!i2c_master::get_instance().init(config_.i2c_sda, config_.i2c_scl, config_.i2c_freq)) {
        ESP_LOGE(TAG, "Failed to initialize I2C master");
        return false;
    }
    // scan_i2c(); // Uncomment to scan I2C bus to find connected devices
;
    imu_ = std::make_unique<bmi_160>(i2c_master::get_instance());
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

    pwm_controller::config esc_timer_config {
        .frequency = 50,              // 50Hz for ESC
        .resolution_bits = 14,        // 14-bit resolution
        .timer_num = LEDC_TIMER_0,    // Different timer for ESC
        .speed_mode = LEDC_LOW_SPEED_MODE
    };

    if (!pwm_controller::getInstance().initTimer(esc_timer_config)) {
        return false;
    }
    ESP_LOGI(TAG, "PWM timer initialized successfully");

    // Initialize servos
    esc_controller::config motor_config {
        .pin = this->config_.motor1_pin,
        .channel = LEDC_CHANNEL_0,
        .timer_num = LEDC_TIMER_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .min_throttle_ms = 1.0f,  // 1ms
        .max_throttle_ms = 2.0f   // 2ms
    };

    const std::array<gpio_num_t, 4> motor_pins = {
        config_.motor1_pin, config_.motor2_pin, 
        config_.motor3_pin, config_.motor4_pin
    };

    const std::array<ledc_channel_t, 4> motor_channels = {
        LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3
    };

    for (size_t i = 0; i < motors_.size(); ++i) {
        motor_config.pin = motor_pins[i];
        motor_config.channel = motor_channels[i];

        motors_[i] = std::make_unique<esc_controller>(motor_config);
        if (!motors_[i]->init()) {
            ESP_LOGE(TAG, "Failed to initialize motor %d", i);
            return false;
        }
    }
    ESP_LOGI(TAG, "Motors initialized successfully");


    pitch_attitude_pid_ = std::make_unique<pid_controller>(pid_configs_.pitch_attitude);
    yaw_attitude_pid_ = std::make_unique<pid_controller>(pid_configs_.yaw_attitude);
    roll_rate_pid_ = std::make_unique<pid_controller>(pid_configs_.roll_rate);
    pitch_rate_pid_ = std::make_unique<pid_controller>(pid_configs_.pitch_rate);
    yaw_rate_pid_ = std::make_unique<pid_controller>(pid_configs_.yaw_rate);
    
    return true;
}

void quad_fcs::imu_task(void* param) {
    auto& system = *static_cast<quad_fcs*>(param);
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

void quad_fcs::sensor_fusion_task(void* param) {
    ESP_ERROR_CHECK(esp_task_wdt_add(xTaskGetCurrentTaskHandle()));

    auto& system = *static_cast<quad_fcs*>(param);
    const float RAD_TO_DEG = 180.0f / M_PI;
    const float TICK_TO_SEC = 1.0f / configTICK_RATE_HZ;

    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for good IMU readings

    TickType_t last_wake_time;
    last_wake_time = xTaskGetTickCount();

    const TickType_t task_period = pdMS_TO_TICKS(10);  // 100Hz

    static int log_counter = 0;
    while (true) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        
        bmi_160::mpu_reading imu_reading;
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

void quad_fcs::barometer_task(void* param) {
    auto& system = *static_cast<quad_fcs*>(param);

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

void quad_fcs::mag_task(void* param) {
    auto& system = *static_cast<quad_fcs*>(param);

    while (true) {
        auto mag_reading = system.mag_->read();
        // ESP_LOGI(TAG, "Magnetometer reading: (%.2f, %.2f, %.2f)",
        //     mag_reading.mag[0], mag_reading.mag[1], mag_reading.mag[2]
        // );

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void quad_fcs::attitude_control_task(void* param) {
    auto& system = *static_cast<quad_fcs*>(param);
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

void quad_fcs::rate_control_task(void* param) {
    auto& system = *static_cast<quad_fcs*>(param);
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
        bmi_160::mpu_reading gyro_data;
        xSemaphoreTake(system.imu_data_.mutex, pdMS_TO_TICKS(5));
        gyro_data = system.imu_data_.latest_reading;
        xSemaphoreGive(system.imu_data_.mutex);
        
        // roll_rate_sp = mapped_channels.yaw_rate_rad_s;
        roll_rate_sp = 0.0;
        pitch_rate_sp = 0.0f;
        yaw_rate_sp = 0.0f;


        // Run rate PIDs
        float roll_thrust = system.roll_rate_pid_->update(
            roll_rate_sp, gyro_data.gyro[1], 0.005f);

        float pitch_thrust = -system.pitch_rate_pid_->update(
            pitch_rate_sp, -gyro_data.gyro[0], 0.005f);

        float yaw_thrust = -system.yaw_rate_pid_->update(
            yaw_rate_sp, -gyro_data.gyro[2], 0.005f);


        // ESP_LOGI(TAG, "motor: %2f%%, gyro: %.2f, %.2f, %.2f; pitch_th: %.2f, yaw_th: %.2f; roll_th: %.2f,  comp_pitch: %.2f, comp_yaw: %.2f",
        //     throttle_percent, gyro_data.gyro[0], gyro_data.gyro[1], gyro_data.gyro[2], pitch_thrust, yaw_thrust, roll_thrust, comp_pitch, comp_yaw
        // );
        // if (tick++ % 8 == 0) {
            // ESP_LOGI(TAG, "Coupled Desired Rates: %.2f, %.2f; gyro: %.2f, %.2f, %.2f; pitch_thrust: %.2f, yaw_thrust: %.2f, K_PREC: %.2f",
            //     pitch_rate_sp_coupled, yaw_rate_sp_coupled,
            //     gyro_data.gyro[0], gyro_data.gyro[1], gyro_data.gyro[2],
            //     pitch_thrust, yaw_thrust, K_PRECESSION
            // );
            // ESP_LOGI(TAG, "Rate sp: %2.2f, %2.2f, %2.2f; throttle: %2.2f; Servo commands: %2.2f, %2.2f, %2.2f, %2.2f",
            //     roll_rate_sp, pitch_rate_sp, yaw_rate_sp, throttle_percent,
            //         servo1_cmd, servo2_cmd, servo3_cmd, servo4_cmd
            //     );
        // }

        float armed_min_throttle = 15.0f;

        // map throttle to account for armed min throttle
        // when throttle stick is at 0, motors should be at armed_min_throttle
        float collective_throttle = mapped_channels.throttle_percent * (0.85) + armed_min_throttle;

        pitch_thrust = rc_mapper::map_range_symmetric(
            rc_channels[1], rc_mapper::RC_MIN, rc_mapper::RC_MAX, -3, 3
        );
        roll_thrust = rc_mapper::map_range_symmetric(
            rc_channels[0], rc_mapper::RC_MIN, rc_mapper::RC_MAX, -3, 3
        );
        yaw_thrust = rc_mapper::map_range_symmetric(
            rc_channels[3], rc_mapper::RC_MIN, rc_mapper::RC_MAX, -3, 3
        );

        float motor1_throttle = collective_throttle + pitch_thrust - roll_thrust - yaw_thrust;
        float motor2_throttle = collective_throttle - pitch_thrust - roll_thrust + yaw_thrust;
        float motor3_throttle = collective_throttle + pitch_thrust + roll_thrust + yaw_thrust;
        float motor4_throttle = collective_throttle - pitch_thrust + roll_thrust - yaw_thrust;
        

        if (mapped_channels.armed) {
            motor1_throttle = std::clamp(motor1_throttle, armed_min_throttle, 100.0f);
            motor2_throttle = std::clamp(motor2_throttle, armed_min_throttle, 100.0f);
            motor3_throttle = std::clamp(motor3_throttle, armed_min_throttle, 100.0f);
            motor4_throttle = std::clamp(motor4_throttle, armed_min_throttle, 100.0f);
        } else {
            motor1_throttle = 0.0f;
            motor2_throttle = 0.0f;
            motor3_throttle = 0.0f;
            motor4_throttle = 0.0f;
        }

        if (tick++ % 10 == 0) {
            ESP_LOGI(TAG, "Motor commands: %2.2f, %2.2f, %2.2f, %2.2f; RC Throttle: %2.2f, Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
                motor1_throttle, motor2_throttle, motor3_throttle, motor4_throttle, collective_throttle, roll_thrust, pitch_thrust, yaw_thrust
            );
        }

        system.motors_[0]->set_throttle(motor1_throttle);
        system.motors_[1]->set_throttle(motor2_throttle);
        system.motors_[2]->set_throttle(motor3_throttle);
        system.motors_[3]->set_throttle(motor4_throttle);

        vTaskDelayUntil(&last_wake_time, period);
    }
}

void quad_fcs::rc_receiver_task(void* param) {
    auto* fcs = static_cast<quad_fcs*>(param);
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

                    if (++tick % 10 == 0) {
                        ESP_LOGI(TAG, "Channels: %d, %d, %d, %d, %d, %d, %d, %d, %2d, %2d",
                            temp_channels[0], temp_channels[1], temp_channels[2], temp_channels[3],
                            temp_channels[4], temp_channels[5], temp_channels[6], temp_channels[7],
                            temp_channels[8], temp_channels[9]
                        );
                    }
                }
            }

        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool quad_fcs::start() {

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