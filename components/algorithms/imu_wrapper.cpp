#include <algorithm>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "ism330.h"
#include "types.h"

#include "imu_wrapper.h"

esp_err_t IMUWrapper::init(i2c_master_bus_handle_t bus) {
    // Initialize IMU and connect to I2C bus
    esp_log_level_set("imu", ESP_LOG_ERROR);

    Ism330::Config cfg;
    cfg.bus            = bus;
    cfg.odr            = Ism330::Odr::Hz417;
    cfg.sample_rate_hz = 200;
    cfg.fusion         = Ism330::FusionType::Mahony;
    cfg.mahony_kp      = 0.5f; // TODO -- tune btwn 0.1 to 1.0

    esp_err_t err = this->imu_.init(cfg);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Initialized IMU\n");

    // Calibrate IMU
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("Calibrating IMU, do not move\n");

    err = this->imu_.calibrate(200);
    if (err != ESP_OK) {
        return err;
    }

    printf("IMU calibration complete\n");

    return ESP_OK;
}

esp_err_t IMUWrapper::update(const float dt_s, SharedData& out_data) {
    if (imu_.getAngles(angles) != ESP_OK) {
        printf("Failed to read IMU\n");
        return ESP_FAIL;
    }
    return ESP_OK;
}