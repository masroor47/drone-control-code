#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"


#define I2C_MASTER_SCL_IO           22      // SCL pin
#define I2C_MASTER_SDA_IO           21      // SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0    // I2C master number
#define I2C_MASTER_FREQ_HZ          400000  // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0       // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0       

#define MPU6050_ADDR               0x68    // MPU6050 address
#define BMP280_ADDR                0x77    // BMP280 address

// BMP280 registers
#define BMP280_REG_TEMP_XLSB    0xFC
#define BMP280_REG_TEMP_LSB     0xFB
#define BMP280_REG_TEMP_MSB     0xFA
#define BMP280_REG_PRESS_XLSB   0xF9
#define BMP280_REG_PRESS_LSB    0xF8
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_HUM     0xF2
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_ID           0xD0

static const char *TAG = "Sensors";

struct bmp280_calib_param {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    int32_t  t_fine;
};


esp_err_t i2c_master_init(void) {
    ESP_LOGI(TAG, "Initializing I2C master on pins - SDA: %d, SCL: %d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    
    i2c_config_t conf = {
        mode: I2C_MODE_MASTER,
        sda_io_num: I2C_MASTER_SDA_IO,
        scl_io_num: I2C_MASTER_SCL_IO,
        sda_pullup_en: GPIO_PULLUP_ENABLE,
        scl_pullup_en: GPIO_PULLUP_ENABLE,
        master: {
            clk_speed: I2C_MASTER_FREQ_HZ
        },
        clk_flags: 0
    };

    esp_err_t err = i2c_param_config(static_cast<i2c_port_t>(I2C_MASTER_NUM), &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(static_cast<i2c_port_t>(I2C_MASTER_NUM), conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

static struct bmp280_calib_param calib_param;

static esp_err_t bmp280_read_calibration(void) {
    uint8_t data[24];
    
    // Read compensation parameters
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x88, true);  // Starting register for calibration data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 24, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return ret;

    // Store calibration data
    calib_param.dig_T1 = (uint16_t)(data[1] << 8) | data[0];
    calib_param.dig_T2 = (int16_t)(data[3] << 8) | data[2];
    calib_param.dig_T3 = (int16_t)(data[5] << 8) | data[4];
    calib_param.dig_P1 = (uint16_t)(data[7] << 8) | data[6];
    calib_param.dig_P2 = (int16_t)(data[9] << 8) | data[8];
    calib_param.dig_P3 = (int16_t)(data[11] << 8) | data[10];
    calib_param.dig_P4 = (int16_t)(data[13] << 8) | data[12];
    calib_param.dig_P5 = (int16_t)(data[15] << 8) | data[14];
    calib_param.dig_P6 = (int16_t)(data[17] << 8) | data[16];
    calib_param.dig_P7 = (int16_t)(data[19] << 8) | data[18];
    calib_param.dig_P8 = (int16_t)(data[21] << 8) | data[20];
    calib_param.dig_P9 = (int16_t)(data[23] << 8) | data[22];
    
    return ESP_OK;
}

// Initialize BMP280
static esp_err_t bmp280_init(void) {
    i2c_cmd_handle_t cmd;
    esp_err_t ret;

    // First, verify we can communicate with the device by reading the ID register
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_ID, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true);
    uint8_t id;
    i2c_master_read_byte(cmd, &id, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280 ID: %s", esp_err_to_name(ret));
        return ret;
    }

    if (id != 0x58) {  // BMP280 ID should be 0x58
        ESP_LOGE(TAG, "Unexpected BMP280 ID: 0x%02x", id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BMP280 ID verified successfully: 0x%02x", id);

    // Reset the device
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_RESET, true);
    i2c_master_write_byte(cmd, 0xB6, true);  // Reset value
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset BMP280: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BMP280 reset successful");
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for reset to complete

    // Read calibration data
    ret = bmp280_read_calibration();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280 calibration: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BMP280 calibration read successful");

    // Configure the sensor
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CTRL_MEAS, true);
    i2c_master_write_byte(cmd, 0x57, true);  // 0b01010111
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BMP280: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BMP280 configuration successful");
    return ESP_OK;
}

// Read temperature and pressure
static esp_err_t bmp280_read_data(float* temperature, float* pressure) {
    uint8_t data[6];
    int32_t adc_T, adc_P;
    int32_t var1, var2, t_fine, p;
    
    // Read raw data
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_PRESS_MSB, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return ret;

    // Calculate temperature
    adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);
    var1 = ((((adc_T >> 3) - ((int32_t)calib_param.dig_T1 << 1))) * 
            ((int32_t)calib_param.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_param.dig_T1)) * 
              ((adc_T >> 4) - ((int32_t)calib_param.dig_T1))) >> 12) * 
            ((int32_t)calib_param.dig_T3)) >> 14;
    t_fine = var1 + var2;
    *temperature = (t_fine * 5 + 128) >> 8;
    *temperature /= 100.0f;

    // Calculate pressure
    adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
    var1 = (((int32_t)t_fine) >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_param.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib_param.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)calib_param.dig_P4) << 16);
    var1 = (((calib_param.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + 
            ((((int32_t)calib_param.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)calib_param.dig_P1)) >> 15);
    
    if (var1 == 0) return ESP_FAIL;  // Avoid division by zero
    
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t)var1);
    } else {
        p = (p / (uint32_t)var1) * 2;
    }
    
    var1 = (((int32_t)calib_param.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)calib_param.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + calib_param.dig_P7) >> 4));
    
    *pressure = p / 100.0f;  // Convert to hPa
    
    return ESP_OK;
}

static int16_t accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
static int16_t gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;


class MPU6050Filter {
private:
    // Filter coefficients
    const float alpha = 0.1; // Adjust between 0 and 1 (higher = more smoothing)
    
    // Previous values
    float filtered_accel_x = 0, filtered_accel_y = 0, filtered_accel_z = 0;
    float filtered_gyro_x = 0, filtered_gyro_y = 0, filtered_gyro_z = 0;
    
public:
    void filter(int16_t raw_ax, int16_t raw_ay, int16_t raw_az,
                int16_t raw_gx, int16_t raw_gy, int16_t raw_gz,
                float* ax, float* ay, float* az,
                float* gx, float* gy, float* gz) {
        // Convert to float and scale to real units
        const float accel_scale = 1.0f / 16384.0f; // For ±2g range
        const float gyro_scale = 1.0f / 131.0f;    // For ±250°/s range
        
        // Apply low-pass filter
        filtered_accel_x = alpha * filtered_accel_x + (1 - alpha) * raw_ax * accel_scale;
        filtered_accel_y = alpha * filtered_accel_y + (1 - alpha) * raw_ay * accel_scale;
        filtered_accel_z = alpha * filtered_accel_z + (1 - alpha) * raw_az * accel_scale;
        
        filtered_gyro_x = alpha * filtered_gyro_x + (1 - alpha) * raw_gx * gyro_scale;
        filtered_gyro_y = alpha * filtered_gyro_y + (1 - alpha) * raw_gy * gyro_scale;
        filtered_gyro_z = alpha * filtered_gyro_z + (1 - alpha) * raw_gz * gyro_scale;
        
        // Output filtered values
        *ax = filtered_accel_x;
        *ay = filtered_accel_y;
        *az = filtered_accel_z;
        *gx = filtered_gyro_x;
        *gy = filtered_gyro_y;
        *gz = filtered_gyro_z;
    }
};

static MPU6050Filter filter;


// MPU6050 initialization
static esp_err_t mpu6050_init(void)
{
    uint8_t cmd[] = {0x6B, 0x00}; // Power management register, wake up MPU6050
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd_handle, cmd, sizeof(cmd), true);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

// Read MPU6050 data
static esp_err_t mpu6050_read_data(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z,
                                  int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z)
{
    uint8_t data[14];
    
    // Start reading from register 0x3B (ACCEL_XOUT_H)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    // Read the data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        *accel_x = (data[0] << 8) | data[1];
        *accel_y = (data[2] << 8) | data[3];
        *accel_z = (data[4] << 8) | data[5];
        *gyro_x = (data[8] << 8) | data[9];
        *gyro_y = (data[10] << 8) | data[11];
        *gyro_z = (data[12] << 8) | data[13];
    }
    
    return ret;
}

static esp_err_t mpu6050_calibrate(int16_t* accel_offset_x, int16_t* accel_offset_y, int16_t* accel_offset_z,
                                  int16_t* gyro_offset_x, int16_t* gyro_offset_y, int16_t* gyro_offset_z) {
    const int num_samples = 1000;
    int32_t accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
    int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    
    // Collect samples
    for (int i = 0; i < num_samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        esp_err_t ret = mpu6050_read_data(&ax, &ay, &az, &gx, &gy, &gz);
        if (ret != ESP_OK) return ret;
        
        accel_x_sum += ax;
        accel_y_sum += ay;
        accel_z_sum += az - 16384; // Remove 1g from z-axis
        gyro_x_sum += gx;
        gyro_y_sum += gy;
        gyro_z_sum += gz;
        
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    // Calculate averages
    *accel_offset_x = -(accel_x_sum / num_samples);
    *accel_offset_y = -(accel_y_sum / num_samples);
    *accel_offset_z = -(accel_z_sum / num_samples);
    *gyro_offset_x = -(gyro_x_sum / num_samples);
    *gyro_offset_y = -(gyro_y_sum / num_samples);
    *gyro_offset_z = -(gyro_z_sum / num_samples);
    
    return ESP_OK;
}

// Sensor reading task
// void sensor_reading_task(void *pvParameters)
// {
//     int16_t accel_x, accel_y, accel_z;
//     int16_t gyro_x, gyro_y, gyro_z;
    
//     // Initialize I2C
//     ESP_ERROR_CHECK(i2c_master_init());
//     ESP_LOGI(TAG, "I2C initialized successfully");
    
//     // Initialize MPU6050
//     ESP_ERROR_CHECK(mpu6050_init());
//     ESP_LOGI(TAG, "MPU6050 initialized successfully");
    
//     TickType_t last_wake_time = xTaskGetTickCount();
//     const TickType_t sample_period = pdMS_TO_TICKS(10); // 100Hz sampling
    
//     while (1) {
//         esp_err_t ret = mpu6050_read_data(&accel_x, &accel_y, &accel_z,
//                                          &gyro_x, &gyro_y, &gyro_z);
                                         
//         if (ret == ESP_OK) {
//             ESP_LOGI(TAG, "Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d",
//                     accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
//         } else {
//             ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
//         }
        
//         // Precise timing for the control loop
//         vTaskDelayUntil(&last_wake_time, sample_period);
//     }
// }

void sensor_reading_task(void *pvParameters)
{
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float temperature, pressure;
    
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    // Initialize MPU6050
    ESP_ERROR_CHECK(mpu6050_init());
    ESP_LOGI(TAG, "MPU6050 initialized successfully");

    // Initialize BMP280
    ESP_ERROR_CHECK(bmp280_init());
    ESP_LOGI(TAG, "BMP280 initialized successfully");
    
    // Perform calibration
    ESP_LOGI(TAG, "Starting calibration. Keep the sensor still...");
    ESP_ERROR_CHECK(mpu6050_calibrate(&accel_offset_x, &accel_offset_y, &accel_offset_z,
                                     &gyro_offset_x, &gyro_offset_y, &gyro_offset_z));
    ESP_LOGI(TAG, "Calibration complete");
    ESP_LOGI(TAG, "Offsets - Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d",
             accel_offset_x, accel_offset_y, accel_offset_z,
             gyro_offset_x, gyro_offset_y, gyro_offset_z);
    
    // Variables for filtered values
    float filtered_ax, filtered_ay, filtered_az;
    float filtered_gx, filtered_gy, filtered_gz;
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t sample_period = pdMS_TO_TICKS(1); // 100Hz sampling
    
    while (1) {
        esp_err_t ret = mpu6050_read_data(&accel_x, &accel_y, &accel_z,
                                         &gyro_x, &gyro_y, &gyro_z);
                                         
        if (ret == ESP_OK) {
            // Apply calibration offsets
            accel_x += accel_offset_x;
            accel_y += accel_offset_y;
            accel_z += accel_offset_z;
            gyro_x += gyro_offset_x;
            gyro_y += gyro_offset_y;
            gyro_z += gyro_offset_z;
            
            // Apply filtering
            filter.filter(accel_x, accel_y, accel_z,
                        gyro_x, gyro_y, gyro_z,
                        &filtered_ax, &filtered_ay, &filtered_az,
                        &filtered_gx, &filtered_gy, &filtered_gz);

            // Read BMP280
            ret = bmp280_read_data(&temperature, &pressure);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Temp: %.2f°C, Pressure: %.1f hPa", temperature, pressure);
            } else {
                ESP_LOGE(TAG, "Failed to read BMP280: %s", esp_err_to_name(ret));
            }
            
            // Log both raw (calibrated) and filtered values
            ESP_LOGI(TAG, "Raw: Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d",
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
                    
            ESP_LOGI(TAG, "Filtered: Accel: X=%.2f Y=%.2f Z=%.2f | Gyro: X=%.2f Y=%.2f Z=%.2f",
                    filtered_ax, filtered_ay, filtered_az,
                    filtered_gx, filtered_gy, filtered_gz);
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        }
        
        // Precise timing for the control loop
        vTaskDelayUntil(&last_wake_time, sample_period);
    }
}



extern "C" void app_main(void)
{
    // Create sensor reading task with high priority
    // Increase stack size significantly (8KB)
    xTaskCreate(sensor_reading_task, "sensor_reading_task", 8192,
            NULL, configMAX_PRIORITIES - 1, NULL);
}
