#include <memory>
#include "esp_log.h"
#include "driver/gpio.h"
#include "control/flight_control_system.hpp"
#include "control/quad_fcs.hpp"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    // static std::unique_ptr<flight_control_system> system;
    // flight_control_system::config config {
    //     .i2c_sda = GPIO_NUM_21,
    //     .i2c_scl = GPIO_NUM_22,
    //     .i2c_freq = 400000,
    //     .servo1_pin = GPIO_NUM_15,
    //     .servo2_pin = GPIO_NUM_12,
    //     .servo3_pin = GPIO_NUM_25,
    //     .servo4_pin = GPIO_NUM_32,
    //     .rc_receiver = {
    //         .rx_pin = GPIO_NUM_16,
    //         .tx_pin = GPIO_NUM_17,
    //         .uart_num = UART_NUM_2,
    //         .baud_rate = 420000
    //     },
    // };
    // system = std::make_unique<flight_control_system>(config);

    // Quadcopter
    static std::unique_ptr<quad_fcs> system;
    quad_fcs::config config {
        .i2c_sda = GPIO_NUM_21,
        .i2c_scl = GPIO_NUM_22,
        .i2c_freq = 400000,
        .motor1_pin = GPIO_NUM_12,
        .motor2_pin = GPIO_NUM_25,
        .motor3_pin = GPIO_NUM_32,
        .motor4_pin = GPIO_NUM_15,
        .rc_receiver = {
            .rx_pin = GPIO_NUM_16,
            .tx_pin = GPIO_NUM_17,
            .uart_num = UART_NUM_2,
            .baud_rate = 420000
        },
    };

    system = std::make_unique<quad_fcs>(config);
    
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
