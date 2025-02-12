#pragma once
#include "freertos/FreeRTOS.h"

#include "esp_log.h"

#include <memory>

#include "control/pid_controller.hpp"
#include "control/pid_configs.hpp"
#include "control/flight_control_config.hpp"
#include "sensors/mpu_6050.hpp"
#include "sensors/bmp_280.hpp"
#include "sensors/gy_271.hpp"
#include "actuators/servo.hpp"
#include "actuators/esc_controller.hpp"

class flight_control_system {
public:
    struct config {
        gpio_num_t i2c_sda;
        gpio_num_t i2c_scl;
        uint32_t i2c_freq;
        gpio_num_t servo1_pin;
        gpio_num_t servo2_pin;
        gpio_num_t servo3_pin;
        gpio_num_t servo4_pin;
    };

    struct attitude_estimate {
        float roll;    // degrees
        float pitch;   // degrees
        float yaw;     // degrees
        TickType_t timestamp;  // microseconds
    };

    struct protected_attitude_data {
        attitude_estimate latest_estimate;
        SemaphoreHandle_t mutex;
    };

    struct protected_imu_data {
        mpu_6050::mpu_reading latest_reading;
        SemaphoreHandle_t mutex;
    };

    struct protected_baro_data {
        bmp_280::reading latest_reading;
        SemaphoreHandle_t mutex;
    };

    struct protected_rate_setpoint_data {
        struct {
            float roll_rate;    // deg/s
            float pitch_rate;   // deg/s
            float yaw_rate;     // deg/s
        } setpoints;
        SemaphoreHandle_t mutex;
    };


    explicit flight_control_system(const config& cfg)
        : config_(cfg),
          imu_(nullptr),
          barometer_(nullptr),
          servos_(),
          pid_configs_(flight_control_config::default_pid_configs()) {
            imu_data_.mutex = xSemaphoreCreateMutex();
            barometer_data_.mutex = xSemaphoreCreateMutex();
            attitude_data_.mutex = xSemaphoreCreateMutex();
            rate_setpoint_data_.mutex = xSemaphoreCreateMutex();
          }
        
    ~flight_control_system() {
        if (imu_data_.mutex != nullptr) {
            vSemaphoreDelete(imu_data_.mutex);
        }
        if (barometer_data_.mutex != nullptr) {
            vSemaphoreDelete(barometer_data_.mutex);
        }
        if (attitude_data_.mutex != nullptr) {
            vSemaphoreDelete(attitude_data_.mutex);
        }
        if (rate_setpoint_data_.mutex != nullptr) {
            vSemaphoreDelete(rate_setpoint_data_.mutex);
        }
        ESP_LOGI("FlightControl", "Shutting down flight control system");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    bool init();
    bool start();

private:
    static constexpr char* TAG = "FlightControl";
    const config config_;
    std::unique_ptr<mpu_6050> imu_;
    std::unique_ptr<bmp_280> barometer_;
    std::unique_ptr<gy_271> mag_;
    std::array<std::unique_ptr<servo>, 4> servos_;
    std::unique_ptr<esc_controller> esc_;

    std::unique_ptr<pid_controller> pitch_attitude_pid_;
    std::unique_ptr<pid_controller> yaw_attitude_pid_;
    
    // Rate PIDs
    std::unique_ptr<pid_controller> roll_rate_pid_;
    std::unique_ptr<pid_controller> pitch_rate_pid_;
    std::unique_ptr<pid_controller> yaw_rate_pid_;
    
    // PID configurations
    pid_configs pid_configs_;

    protected_imu_data imu_data_;
    protected_baro_data barometer_data_;
    protected_attitude_data attitude_data_;
    protected_rate_setpoint_data rate_setpoint_data_;

    struct filter_state {
        // pitch roll yaw
        float pitch = 0.0f;
        float roll = 0.0f;
        float yaw = 0.0f;
        TickType_t last_update = 0;
    } filter_state_;
    
    struct filter_params {
        float alpha = 0.04f;
        float gyro_scale = 1.0f;
    } filter_params_;

    
    static void imu_task(void* param);
    TaskHandle_t imu_task_handle_;

    static void barometer_task(void* param);
    TaskHandle_t barometer_task_handle_;

    static void mag_task(void* param);
    TaskHandle_t mag_task_handle_;

    static void sensor_fusion_task(void* param);
    TaskHandle_t sensor_fusion_task_handle_;

    static void attitude_control_task(void* param);
    TaskHandle_t attitude_control_task_handle_;

    static void rate_control_task(void* param);
    TaskHandle_t rate_control_task_handle_;

    static void control_task(void* param);
    TaskHandle_t control_task_handle_;

    void scan_i2c();
};