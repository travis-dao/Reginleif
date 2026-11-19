#include "esp_log.h"
#include "esp_timer.h"
#include "imu_subsystem.h"
#include <cmath>

static const char* TAG = "stabilizer";

Stabilizer::Stabilizer(ImuMailbox& mailbox, const Config& config)
    : mailbox_(mailbox), cfg_(config) {}

void Stabilizer::init(i2c_master_bus_handle_t bus) {
    esp_log_level_set("imu", ESP_LOG_ERROR);

    Ism330::Config cfg;
    cfg.bus            = bus;
    cfg.odr            = Ism330::Odr::Hz417;
    cfg.sample_rate_hz = cfg_.sample_rate_hz;
    cfg.fusion         = Ism330::FusionType::Mahony;
    cfg.mahony_kp      = cfg_.mahony_kp;

    ESP_ERROR_CHECK(imu_.init(cfg));   // startup only: abort here is fine
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Initialized Imu\n");

    vTaskDelay(pdMS_TO_TICKS(500));
    printf("Calibrating Imu, do not move\n");
    ESP_ERROR_CHECK(imu_.calibrate(cfg_.calib_samples));
    printf("Imu calibration complete\n");
}

void Stabilizer::update() {
    ImuSample s;
    s.stamp_us = static_cast<uint32_t>(esp_timer_get_time());

    // Runtime: never abort on a transient I2C error. Publish valid=false instead
    // so the motion side can e-stop deliberately.
    const esp_err_t err = imu_.getAngles(angles_deg_);
    if (err == ESP_OK) {
        gyro_dps_ = imu_.lastGyro();   // deg/s about x, y, z

        s.roll  = angles_deg_.roll  * DEG_TO_RAD;
        s.pitch = angles_deg_.pitch * DEG_TO_RAD;
        s.yaw   = angles_deg_.yaw   * DEG_TO_RAD;
        s.gyro_body = gyro_dps_ * DEG_TO_RAD;

        s.valid = std::isfinite(s.roll) && std::isfinite(s.pitch) && std::isfinite(s.yaw) &&
                  s.gyro_body.allFinite();
    } else {
        s.valid = false;
    }

    if (!s.valid) {
        if ((error_count_++ % 100) == 0) {
            ESP_LOGW(TAG, "IMU read failed (%s), total errors %u",
                     esp_err_to_name(err), (unsigned)error_count_);
        }
    }

    mailbox_.publish(s);
}

void Stabilizer::taskEntry(void* self) {
    auto* st = static_cast<Stabilizer*>(self);

    // At 200 Hz you want CONFIG_FREERTOS_HZ=1000; with 100 Hz ticks this rounds to 100 Hz
    // and no longer matches the fusion sample_rate_hz.
    TickType_t period = configTICK_RATE_HZ / st->cfg_.sample_rate_hz;
    if (period < 1) period = 1;

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        st->update();
        vTaskDelayUntil(&last, period);
    }
}