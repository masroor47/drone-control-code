#include <memory>
#include "esp_log.h"
#include "driver/gpio.h"
#include "control/flight_control_system.hpp"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    static std::unique_ptr<flight_control_system> system;

    flight_control_system::config config {
        .i2c_sda = GPIO_NUM_21,
        .i2c_scl = GPIO_NUM_22,
        .i2c_freq = 400000,
        .servo_pin = GPIO_NUM_18
    };

    system = std::make_unique<flight_control_system>(config);
    
    if (!system->init()) {
        ESP_LOGE(TAG, "Failed to initialize flight control system");
        return;
    }
    ESP_LOGI(TAG, "Flight control system initialized successfully");

    if (!system->start()) {
        ESP_LOGE(TAG, "Failed to start flight control system");
        return;
    }

}
