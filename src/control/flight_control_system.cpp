#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include <memory>

#include "control/flight_control_system.hpp"
#include "drivers/i2c_master.hpp"
#include "sensors/mpu_6050.hpp"
#include "sensors/bmp_280.hpp"
#include "drivers/pwm_controller.hpp"
#include "actuators/servo.hpp"
#include "actuators/esc_controller.hpp"
#include "utils/thread_safe_queue.hpp"




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
    barometer_ = std::make_unique<bmp_280>(i2c_master::get_instance());
    mag_ = std::make_unique<gy_271>(i2c_master::get_instance());

    if (!imu_->init()) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 initialized successfully");

    if (!barometer_->init()) {
        ESP_LOGE(TAG, "Failed to initialize BMP280");
        return false;
    }
    ESP_LOGI(TAG, "BMP280 initialized successfully");

    if (!mag_->init()) {
        ESP_LOGE(TAG, "Failed to initialize GY271");
        return false;
    }
    ESP_LOGI(TAG, "GY271 initialized successfully");


    // imu_queue_ = std::make_unique<thread_safe_queue<mpu_6050::mpu_reading>>();
    // ESP_LOGI(TAG, "IMU queue created, about to initialize pwm");
    // barometer_queue_ = std::make_unique<thread_safe_queue<bmp_280::reading>>();
    // ESP_LOGI(TAG, "Baro queue created, about to initialize pwm");

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
    ESP_LOGI(TAG, "Servos initialized successfully");

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

    ESP_LOGI(TAG, "ESC initialized successfully");
    
    return true;
}

void flight_control_system::imu_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);

    while (true) {
        auto imu_reading = system.imu_->read();
        if (xSemaphoreTake(system.imu_data_.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            system.imu_data_.latest_reading = imu_reading;
            xSemaphoreGive(system.imu_data_.mutex);
        }
        // ESP_LOGI(TAG, "IMU reading: Accel: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f)",
        //     imu_reading.accel[0], imu_reading.accel[1], imu_reading.accel[2],
        //     imu_reading.gyro[0], imu_reading.gyro[1], imu_reading.gyro[2]
        // );

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void flight_control_system::sensor_fusion_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);
    const float RAD_TO_DEG = 180.0f / M_PI;

    while (true) {
        mpu_6050::mpu_reading imu_reading;
        if (xSemaphoreTake(system.imu_data_.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            imu_reading = system.imu_data_.latest_reading;
            xSemaphoreGive(system.imu_data_.mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint64_t current_time = esp_timer_get_time();
        float dt = (current_time - system.filter_state_.last_update) / 1e6;

        if (system.filter_state_.last_update == 0) {
            system.filter_state_.last_update = current_time;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        float accel_roll = atan2f(imu_reading.accel[1], imu_reading.accel[2]) * RAD_TO_DEG;
        float accel_pitch = atan2f(-imu_reading.accel[0], 
            sqrtf(imu_reading.accel[1] * imu_reading.accel[1] + 
            imu_reading.accel[2] * imu_reading.accel[2])) * RAD_TO_DEG;

        float gyro_roll = system.filter_state_.prev_roll + 
            imu_reading.gyro[0] * system.filter_params_.gyro_scale * dt;
        float gyro_pitch = system.filter_state_.prev_pitch + 
            imu_reading.gyro[1] * system.filter_params_.gyro_scale * dt;
        float gyro_yaw = system.filter_state_.prev_yaw +
            imu_reading.gyro[2] * system.filter_params_.gyro_scale * dt;

        float alpha = system.filter_params_.alpha;
        attitude_estimate new_estimate {
            .roll = (1.0f - alpha) * gyro_roll + alpha * accel_roll,
            .pitch = (1.0f - alpha) * gyro_pitch + alpha * accel_pitch,
            .yaw = gyro_yaw
            .timestamp = current_time
        }

        ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
            new_estimate.roll, new_estimate.pitch, new_estimate.yaw
        );
        
        system.filter_state_.prev_roll = new_estimate.roll;
        system.filter_state_.prev_pitch = new_estimate.pitch;
        system.filter_state_.prev_yaw = new_estimate.yaw;
        system.filter_state_.last_update = current_time;

        if (xSemaphoreTake(system.attitude_data_.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            system.attitude_data_.latest_estimate = new_estimate;
            xSemaphoreGive(system.attitude_data_.mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
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

    BaseType_t ret = xTaskCreate(
        imu_task,
        "imu_task",
        4096,
        this,
        configMAX_PRIORITIES - 2,
        &imu_task_handle_
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return false;
    }

    ret = xTaskCreate(
        sensor_fusion_task,
        "sensor_fusion_task",
        4096,
        this,
        configMAX_PRIORITIES - 2,
        &sensor_fusion_task_handle_
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor fusion task");
        return false;
    }

    ret = xTaskCreate(
        barometer_task,
        "barometer_task",
        4096,
        this,
        configMAX_PRIORITIES - 2,
        &barometer_task_handle_
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create barometer task");
        return false;
    }

    ret = xTaskCreate(
        mag_task,
        "mag_task",
        4096,
        this,
        configMAX_PRIORITIES - 2,
        &mag_task_handle_
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create magnetometer task");
        return false;
    }


    // ret = xTaskCreate(
    //     control_task,
    //     "control_task",
    //     4096,
    //     this,
    //     configMAX_PRIORITIES - 2,
    //     &control_task_handle_
    // );

    // if (ret != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create control task");
    //     return false;
    // }

    return true;
}