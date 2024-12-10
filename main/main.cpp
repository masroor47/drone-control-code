#include <memory>
#include "esp_log.h"
#include "driver/gpio.h"
#include "control/flight_control_system.hpp"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    flight_control_system::config config {
        .i2c_sda = GPIO_NUM_21,
        .i2c_scl = GPIO_NUM_22,
        .i2c_freq = 400000,
        .servo_pin = GPIO_NUM_18
    };

    // Create and initialize flight control system
    auto system = std::make_unique<flight_control_system>(config);
    
    if (!system->init()) {
        ESP_LOGE(TAG, "Failed to initialize flight control system");
        return;
    }

    if (!system->start()) {
        ESP_LOGE(TAG, "Failed to start flight control system");
        return;
    }

}
