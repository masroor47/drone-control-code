#pragma once
#include "freertos/FreeRTOS.h"

#include "esp_log.h"

#include <memory>

#include "sensors/mpu_6050.hpp"
#include "sensors/bmp_280.hpp"
#include "sensors/gy_271.hpp"
#include "actuators/servo.hpp"
#include "actuators/esc_controller.hpp"
#include "test_sequence.hpp"

class flight_control_system {
// std::atomic<bool> shutdown_requested_ = false;
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

    explicit flight_control_system(const config& cfg)
        : config_(cfg),
          imu_(nullptr),
          barometer_(nullptr),
          servos_() {
            imu_data_.mutex = xSemaphoreCreateMutex();
            barometer_data_.mutex = xSemaphoreCreateMutex();
            attitude_data_.mutex = xSemaphoreCreateMutex();
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

    std::unique_ptr<test_sequence> test_sequence_;

    protected_imu_data imu_data_;
    protected_baro_data barometer_data_;
    protected_attitude_data attitude_data_;

    struct filter_state {
        float prev_roll = 0.0f;
        float prev_pitch = 0.0f;
        float prev_yaw = 0.0f;
        TickType_t last_update = 0;
    } filter_state_;
    
    struct filter_params {
        float alpha = 0.05f;
        float gyro_scale = 1.0f;
    } filter_params_;

    static void control_task(void* param);
    static void imu_task(void* param);
    static void barometer_task(void* param);
    static void mag_task(void* param);
    static void test_sequence_task(void* param);
    static void sensor_fusion_task(void* param);
    TaskHandle_t control_task_handle_;
    TaskHandle_t imu_task_handle_;
    TaskHandle_t barometer_task_handle_;
    TaskHandle_t mag_task_handle_;
    TaskHandle_t test_sequence_task_handle_;
    TaskHandle_t sensor_fusion_task_handle_;
    void scan_i2c();
};