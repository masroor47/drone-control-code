#pragma once
#include <memory>
#include "esp_log.h"
#include "sensors/mpu_6050.hpp"
#include "sensors/bmp_280.hpp"
#include "actuators/servo.hpp"
#include "actuators/esc_controller.hpp"
#include "utils/thread_safe_queue.hpp"

class flight_control_system {
// std::atomic<bool> shutdown_requested_ = false;
public:
    struct config {
        gpio_num_t i2c_sda;
        gpio_num_t i2c_scl;
        uint32_t i2c_freq;
        gpio_num_t servo_pin;
    };

    explicit flight_control_system(const config& cfg)
        : config_(cfg),
          imu_(nullptr),
          barometer_(nullptr),
          servo_(nullptr) {}
        
    ~flight_control_system() {
        // shutdown_requested_ = true;
        ESP_LOGI("FlightControl", "Shutting down flight control system");
        // Wait for tasks to finish
        vTaskDelay(pdMS_TO_TICKS(100));  // Give tasks time to stop
        // Then delete tasks if needed
    }

    bool init();
    bool start();

private:
    const config config_;
    std::unique_ptr<mpu_6050> imu_;
    std::unique_ptr<bmp_280> barometer_;
    std::unique_ptr<servo> servo_;
    std::unique_ptr<esc_controller> esc_;
    std::unique_ptr<thread_safe_queue<mpu_6050::mpu_reading>> imu_queue_;
    std::unique_ptr<thread_safe_queue<bmp_280::reading>> barometer_queue_;
    static void sensor_task(void* param);
    static void control_task(void* param);
    TaskHandle_t sensor_task_handle_;
    TaskHandle_t control_task_handle_;
};