#include "control/flight_control_system.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_task_wdt.h"

#include <memory>

#include "esp_log.h"
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
        if (xSemaphoreTake(system.imu_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            system.imu_data_.latest_reading = imu_reading;
            xSemaphoreGive(system.imu_data_.mutex);
        }
        ESP_LOGI(TAG, "IMU reading: Accel: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f)",
            imu_reading.accel[0], imu_reading.accel[1], imu_reading.accel[2],
            imu_reading.gyro[0], imu_reading.gyro[1], imu_reading.gyro[2]
        );

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void flight_control_system::sensor_fusion_task(void* param) {
    ESP_ERROR_CHECK(esp_task_wdt_add(xTaskGetCurrentTaskHandle()));

    auto& system = *static_cast<flight_control_system*>(param);
    const float RAD_TO_DEG = 180.0f / M_PI;
    const float TICK_TO_SEC = 1.0f / configTICK_RATE_HZ;

    TickType_t last_wake_time;
    last_wake_time = xTaskGetTickCount();

    const TickType_t frequency = pdMS_TO_TICKS(10);  // 100Hz

    while (true) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        TickType_t start_time = xTaskGetTickCount();
        TickType_t total_start = xTaskGetTickCount();

        if (frequency > 0) {
            vTaskDelayUntil(&last_wake_time, frequency);
        } else {
            ESP_LOGW(TAG, "Frequency is 0, using vTaskDelay instead");
            vTaskDelay(1);
        }
        
        mpu_6050::mpu_reading imu_reading;
        TickType_t mutex_start = xTaskGetTickCount();
        if (xSemaphoreTake(system.imu_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            TickType_t mutex_time = xTaskGetTickCount() - mutex_start;
            imu_reading = system.imu_data_.latest_reading;
            xSemaphoreGive(system.imu_data_.mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        TickType_t calc_start = xTaskGetTickCount();

        TickType_t current_ticks = xTaskGetTickCount();
        float dt;

        if (system.filter_state_.last_update == 0) {
            system.filter_state_.last_update = current_ticks;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        dt = (float)(current_ticks - system.filter_state_.last_update) * TICK_TO_SEC;

        float accel_pitch = atan2f(imu_reading.accel[2], -imu_reading.accel[1]) * RAD_TO_DEG;  // Now using Z component
        float accel_yaw = atan2f(imu_reading.accel[0], -imu_reading.accel[1]) * RAD_TO_DEG;    // Now using X component

        // For gyro, roll is around the vertical axis (Y axis showing ~-9.8g)
        float gyro_roll = system.filter_state_.prev_roll + 
            imu_reading.gyro[1] * system.filter_params_.gyro_scale * dt;
        float gyro_pitch = system.filter_state_.prev_pitch + 
            imu_reading.gyro[2] * system.filter_params_.gyro_scale * dt;  // Swapped with yaw
        float gyro_yaw = system.filter_state_.prev_yaw +
            imu_reading.gyro[0] * system.filter_params_.gyro_scale * dt;  // Swapped with pitch

        float alpha = system.filter_params_.alpha;
        attitude_estimate new_estimate {
            .roll = gyro_roll,  // Roll can only come from gyro integration
            .pitch = (1.0f - alpha) * gyro_pitch + alpha * accel_pitch,
            .yaw = (1.0f - alpha) * gyro_yaw + alpha * accel_yaw,
            .timestamp = current_ticks
        };

        ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
            new_estimate.roll, new_estimate.pitch, new_estimate.yaw
        );
        
        system.filter_state_.prev_roll = new_estimate.roll;
        system.filter_state_.prev_pitch = new_estimate.pitch;
        system.filter_state_.prev_yaw = new_estimate.yaw;
        system.filter_state_.last_update = current_ticks;

        TickType_t calc_time = xTaskGetTickCount() - calc_start;

        TickType_t second_mutex_start = xTaskGetTickCount();
        if (xSemaphoreTake(system.attitude_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            TickType_t second_mutex_time = xTaskGetTickCount() - second_mutex_start;
            system.attitude_data_.latest_estimate = new_estimate;
            xSemaphoreGive(system.attitude_data_.mutex);
        }
        TickType_t second_mutex_end = xTaskGetTickCount();
        
        TickType_t total_time = second_mutex_end - total_start;
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

    while (true) {
        if (xSemaphoreTake(system.attitude_data_.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            attitude = system.attitude_data_.latest_estimate;
            xSemaphoreGive(system.attitude_data_.mutex);
            // ESP_LOGI(TAG, "Attitude: Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
            //     attitude.roll, attitude.pitch, attitude.yaw
            // );
        }
        system.servos_[0]->set_angle(attitude.roll);
        system.servos_[1]->set_angle(attitude.pitch);
        system.servos_[2]->set_angle(-attitude.roll);
        system.servos_[3]->set_angle(-attitude.pitch);

        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}

void flight_control_system::test_sequence_task(void* param) {
    auto& system = *static_cast<flight_control_system*>(param);

    
}

bool flight_control_system::start() {

    BaseType_t ret = xTaskCreatePinnedToCore(
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

    ret = xTaskCreatePinnedToCore(
        mag_task,
        "mag_task",
        4096,
        this,
        configMAX_PRIORITIES - 6,
        &mag_task_handle_,
        1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create magnetometer task");
        return false;
    }


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

    // ret = xTaskCreatePinnedToCore(
    //     test_sequence_task,
    //     "test_sequence_task",
    //     4096,
    //     this,
    //     configMAX_PRIORITIES - 2,
    //     &test_sequence_task_handle_
    //     0
    // );

    return true;
}