#include <algorithm>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "ism330.h"
#include "types.h"

#include "pid_stabilize.h"

static constexpr float MAX_COMP_DEG = 15.0f;   // max body-tilt correction, per axis
static constexpr float I_MAX        = 10.0f;   // max integral_sum in deg*s

Stabilizer::Stabilizer(const Config& config) {
    this->K_p = config.K_p;
    this->K_i = config.K_i;
    this->K_d = config.K_d;

    this->integral_sum = Vec3 { 0.0f, 0.0f, 0.0f };

    this->rot_mat = Mat3 {{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    }};
}

esp_err_t Stabilizer::init(i2c_master_bus_handle_t bus) {
    // Initialize IMU and connect to I2C bus
    esp_log_level_set("mpu6050", ESP_LOG_ERROR);

    Ism330::Config cfg;
    cfg.bus            = bus;
    cfg.odr            = Ism330::Odr::Hz417;
    cfg.sample_rate_hz = 200;
    cfg.fusion         = Ism330::FusionType::Mahony;
    cfg.mahony_kp      = 0.5f;   // TODO -- tune btwn 0.1 to 1.0

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

esp_err_t Stabilizer::updateImu() {
    if (imu_.getAngles(angles) != ESP_OK) {
        printf("Failed to read IMU\n");
        return ESP_FAIL;
    }
    Vec3 g = imu_.lastGyro();
    gyro_dps_ = Vec3 { g.x, -g.y, 0.0f };  // negate y to match the negated pitch angle
    return ESP_OK;
}

void Stabilizer::updatePID(float dt_s, const Vec3& angles, const Vec3& gyro_dps) {
    if (dt_s <= 0.0f) return;

    Vec3 error = target_angles - angles;
    error.z = 0.0f; // no yaw feedback

    integral_sum = integral_sum + error * dt_s;
    integral_sum.x = std::clamp(integral_sum.x, -I_MAX, I_MAX);
    integral_sum.y = std::clamp(integral_sum.y, -I_MAX, I_MAX);

    Vec3 d_term = (gyro_dps * -1.0f) * K_d;   // gyro measurement = change in deg/sec = derivative

    Vec3 comp_deg = error * K_p + integral_sum * K_i + d_term;
    comp_deg.x = std::clamp(comp_deg.x, -MAX_COMP_DEG, MAX_COMP_DEG);
    comp_deg.y = std::clamp(comp_deg.y, -MAX_COMP_DEG, MAX_COMP_DEG);
    comp_deg.z = 0.0f;

    this->rot_mat = getRotationMatrix(comp_deg * DEG_TO_RAD);
}

esp_err_t Stabilizer::update(const float dt_s, SharedData& out_data) {
    if (updateImu() != ESP_OK) return ESP_FAIL;

    updatePID(dt_s, getAngles(), getGyro());
    out_data.rot_mat = rot_mat;
    return ESP_OK;
}

Mat3 Stabilizer::getRotationMatrix(const Vec3& a) {
    const float cx = cosf(a.x), sx = sinf(a.x);
    const float cy = cosf(a.y), sy = sinf(a.y);

    // R = Rx * Ry (roll, then pitch; no yaw)
    return Mat3 {{
        { cy,      0.0f,  sy      },
        { sx * sy, cx,   -sx * cy },
        {-cx * sy, sx,    cx * cy }
    }};
}