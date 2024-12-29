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

    if (!i2c_master::get_instance().init(config_.i2c_sda, config_.i2c_scl, config_.i2c_freq)) {
        ESP_LOGE("FlightControl", "Failed to initialize I2C master");
        return false;
    }

    // imu_ = std::make_unique<mpu_6050>(i2c_master::get_instance());
    // barometer_ = std::make_unique<bmp_280>(i2c_master::get_instance());

    // if (!imu_->init()) {
    //     ESP_LOGE("FlightControl", "Failed to initialize MPU6050");
    //     return false;
    // }
    // ESP_LOGI("FlightControl", "MPU6050 initialized successfully");

    // if (!barometer_->init()) {
    //     ESP_LOGE("FlightControl", "Failed to initialize BMP280");
    //     return false;
    // }

    // imu_queue_ = std::make_unique<thread_safe_queue<mpu_6050::mpu_reading>>();
    // ESP_LOGI("FlightControl", "IMU queue created, about to initialize pwm");
    // barometer_queue_ = std::make_unique<thread_safe_queue<bmp_280::reading>>();
    // ESP_LOGI("FlightControl", "Baro queue created, about to initialize pwm");

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
    ESP_LOGI("FlightControl", "PWM timers initialized successfully");

    // Initialize servos
    servo::config servo_config {
        .pin = this->config_.servo1_pin,
        .channel = LEDC_CHANNEL_0,
        .timer_num = LEDC_TIMER_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .min_pulse_ms = 1.0f,    // 1ms
        .max_pulse_ms = 2.0f,    // 2ms
        .min_angle = -20.0f,
        .max_angle = 20.0f,
        .calib_offset = 0.0f
    };

    const std::array<gpio_num_t, 4> servo_pins = {
        config_.servo1_pin, config_.servo2_pin, 
        config_.servo3_pin, config_.servo4_pin
    };

    const std::array<ledc_channel_t, 4> servo_channels = {
        LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3
    };

    const std::array<float, 4> calib_offset = {
        -2.0f, 2.0f, -2.0f, -5.0f
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
    ESP_LOGI("FlightControl", "Servos initialized successfully");

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

    ESP_LOGI("FlightControl", "ESC initialized successfully");
    
    return true;
}

void flight_control_system::imu_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);

    while (true) {
        auto imu_reading = system.imu_->read();

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void flight_control_system::barometer_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);

    while (true) {
        auto baro_reading = system.barometer_->read();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void flight_control_system::control_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);
    
    // bool completed_one = false;
    // while(true) {
        // if (auto data = system.imu_queue_->pop(pdMS_TO_TICKS(100))) {
        //     // Process data

        // }
        
    for (auto& servo : system.servos_) {
        servo->set_angle(0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // for (float angle = -20.0f; angle <= 20.0f; angle += 0.1f) {
    //     // for (auto& servo : system.servos_) {
    //         // servo->set_angle(angle);
    //     // }
    //     system.servos_[0]->set_angle(angle);
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }
    // }

    // only 0 and 2 should turn
    system.servos_[0]->set_angle(20.0f);
    system.servos_[2]->set_angle(20.0f);
    vTaskDelay(pdMS_TO_TICKS(500));
    for (auto& servo : system.servos_) {
        servo->set_angle(0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    system.servos_[0]->set_angle(-20.0f);
    system.servos_[2]->set_angle(-20.0f);
    vTaskDelay(pdMS_TO_TICKS(500));
    for (auto& servo : system.servos_) {
        servo->set_angle(0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(500));


    // only 1 and 3 should turn
    system.servos_[1]->set_angle(20.0f);
    system.servos_[3]->set_angle(20.0f);
    vTaskDelay(pdMS_TO_TICKS(500));
    for (auto& servo : system.servos_) {
        servo->set_angle(0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    system.servos_[1]->set_angle(-20.0f);
    system.servos_[3]->set_angle(-20.0f);
    vTaskDelay(pdMS_TO_TICKS(500));
    for (auto& servo : system.servos_) {
        servo->set_angle(0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

bool flight_control_system::start() {

    // BaseType_t ret = xTaskCreate(
    //     imu_task,
    //     "imu_task",
    //     4096,
    //     this,
    //     configMAX_PRIORITIES - 2,
    //     nullptr
    // );

    // if (ret != pdPASS) {
    //     ESP_LOGE("FlightControl", "Failed to create IMU task");
    //     return false;
    // }

    // ret = xTaskCreate(
    //     barometer_task,
    //     "barometer_task",
    //     4096,
    //     this,
    //     configMAX_PRIORITIES - 2,
    //     nullptr
    // );

    // if (ret != pdPASS) {
    //     ESP_LOGE("FlightControl", "Failed to create barometer task");
    //     return false;
    // }

    BaseType_t ret = xTaskCreate(
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