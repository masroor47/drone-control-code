#include <memory>
#include "esp_log.h"

#include "control/flight_control_system.hpp"
#include "drivers/i2c_master.hpp"
#include "sensors/mpu_6050.hpp"
#include "sensors/bmp_280.hpp"
#include "drivers/pwm_controller.hpp"
#include "actuators/servo.hpp"
#include "actuators/esc_controller.hpp"
#include "utils/thread_safe_queue.hpp"

bool flight_control_system::init() {
    imu_ = std::make_unique<mpu_6050>(i2c_master::get_instance());
    barometer_ = std::make_unique<bmp_280>(i2c_master::get_instance());

    if (!imu_->init()) {
        ESP_LOGE("FlightControl", "Failed to initialize MPU6050");
        return false;
    }

    if (!barometer_->init()) {
        ESP_LOGE("FlightControl", "Failed to initialize BMP280");
        return false;
    }

    imu_queue_ = std::make_unique<thread_safe_queue<mpu_6050::mpu_reading>>();
    barometer_queue_ = std::make_unique<thread_safe_queue<bmp_280::reading>>();

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

    // Initialize servo
    servo::config servo_config {
        .pin = GPIO_NUM_18,
        .channel = LEDC_CHANNEL_0,
        .timer_num = LEDC_TIMER_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .min_pulse_ms = 1.0f,    // 1ms
        .max_pulse_ms = 2.0f,    // 2ms
        .min_angle = -90.0f,
        .max_angle = 90.0f
    };

    servo_ = std::make_unique<servo>(servo_config);
    if (!servo_->init()) {
        return false;
    }

    esc_controller::config esc_config {
        .pin = GPIO_NUM_19,
        .channel = LEDC_CHANNEL_1,
        .timer_num = LEDC_TIMER_1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .min_throttle_ms = 1.0f,  // 1ms
        .max_throttle_ms = 2.0f   // 2ms
    };

    esc_ = std::make_unique<esc_controller>(esc_config);
    if (!esc_->init()) {
        return false;
    }

    if (!i2c_master::get_instance().init(config_.i2c_sda, config_.i2c_scl, config_.i2c_freq)) {
        ESP_LOGE("FlightControl", "Failed to initialize I2C master");
        return false;
    }
    return true;
}

void flight_control_system::sensor_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);

    while (true) {
        auto imu_reading = system.imu_->read();
        if (!system.imu_queue_->push(imu_reading)) {
            ESP_LOGE("FlightControl", "Failed to push IMU reading to queue");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void flight_control_system::control_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);
    
    while(true) {
        if (auto data = system.imu_queue_->pop(pdMS_TO_TICKS(100))) {
            // Process data

        }
    }
}


bool flight_control_system::start() {
    BaseType_t ret = xTaskCreate(
        sensor_task,
        "sensor_task",
        8192,
        this,
        configMAX_PRIORITIES - 1,
        &sensor_task_handle_
    );

    if (ret != pdPASS) {
        ESP_LOGE("FlightControl", "Failed to create sensor task");
        return false;
    }

    ret = xTaskCreate(
        control_task,
        "control_task",
        4096,
        this,
        configMAX_PRIORITIES - 2,
        &control_task_handle_
    );

    if (ret != pdPASS) {
        ESP_LOGE("FlightControl", "Failed to create control task");
        return false;
    }

    return true;
}