#include "esp_log.h"
#include "stabilizer.h"
#include <algorithm>
#include <cmath>


static constexpr float MAX_COMP_DEG = 15.0f;
static constexpr float INTEGRAL_MAX = 10.0f;

// Wrap an angle difference to [-180, 180] so yaw takes the shortest path.
static inline float wrapDeg(float a) {
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a - 180.0f;
}

Stabilizer::Stabilizer(const Config& config)
    : target_angles_deg_{0.0f, 0.0f, 0.0f},
      gyro_dps_{0.0f, 0.0f, 0.0f},
      K_p_(config.K_p),
      K_i_(config.K_i),
      K_d_(config.K_d),
      integral_sum_{0.0f, 0.0f, 0.0f}
{
    rot_mat_ = getRotationMatrix(Vec3{0.0f, 0.0f, 0.0f});   // identity until the first update
}

void Stabilizer::init(i2c_master_bus_handle_t bus) {
    // Initialize IMU and connect to I2C bus
    esp_log_level_set("imu", ESP_LOG_ERROR);

    Ism330::Config cfg;
    cfg.bus            = bus;
    cfg.odr            = Ism330::Odr::Hz417;
    cfg.sample_rate_hz = 200;
    cfg.fusion         = Ism330::FusionType::Mahony;
    cfg.mahony_kp      = 0.5f; // TODO -- tune btwn 0.1 to 1.0

    ESP_ERROR_CHECK(imu_.init(cfg));

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Initialized IMU\n");

    // Calibrate IMU
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("Calibrating IMU, do not move\n");

    ESP_ERROR_CHECK(imu_.calibrate(200));

    printf("IMU calibration complete\n");
}

void Stabilizer::updateImu() {
    // Standard convention: right-handed frame, x = roll axis, y = pitch axis,
    // z = yaw axis, positive rotation by the right-hand rule.
    // angles_deg_ = {roll, pitch, yaw} in degrees.
    // lastGyro() reports degrees per second about x, y, z (no sign flips).
    ESP_ERROR_CHECK(imu_.getAngles(angles_deg_));
    gyro_dps_ = imu_.lastGyro();
}

void Stabilizer::updatePID(float dt_s, const Vec3& angles_in_deg, const Vec3& gyro_dps) {
    if (dt_s <= 0.0f) return;

    Vec3 error = target_angles_deg_ - angles_in_deg;
    error.z = wrapDeg(error.z);   // shortest-path yaw error

    Vec3 d_term = gyro_dps * (-1.0f) * K_d_;

    // Conditional anti-windup: don't grow the integral when doing so would
    // push an already-saturated output further into saturation.
    auto updateIntegralAxis = [&](float error_axis, float d_axis, float kp, float ki, float& integral_axis) {
        if (ki == 0.0f) return;

        const float proposed_integral = std::clamp(
            integral_axis + error_axis * dt_s,
            -INTEGRAL_MAX,
            INTEGRAL_MAX
        );

        const float proposed_output =
            kp * error_axis + ki * proposed_integral + d_axis;

        const bool drives_further_into_saturation =
            (proposed_output > MAX_COMP_DEG && ki * error_axis > 0.0f) ||
            (proposed_output < -MAX_COMP_DEG && ki * error_axis < 0.0f);

        if (!drives_further_into_saturation) {
            integral_axis = proposed_integral;
        }
    };

    updateIntegralAxis(error.x, d_term.x, K_p_, K_i_, integral_sum_.x);
    updateIntegralAxis(error.y, d_term.y, K_p_, K_i_, integral_sum_.y);
    updateIntegralAxis(error.z, d_term.z, K_p_, K_i_, integral_sum_.z);

    Vec3 comp_deg = error * K_p_ + integral_sum_ * K_i_ + d_term;

    comp_deg.x = std::clamp(comp_deg.x, -MAX_COMP_DEG, MAX_COMP_DEG);
    comp_deg.y = std::clamp(comp_deg.y, -MAX_COMP_DEG, MAX_COMP_DEG);
    comp_deg.z = std::clamp(comp_deg.z, -MAX_COMP_DEG, MAX_COMP_DEG);

    rot_mat_ = getRotationMatrix(comp_deg * DEG_TO_RAD);
}

void Stabilizer::update(const float dt_s, ImuCorrection& out_data) {
    updateImu();

    updatePID(dt_s, angles(), gyro());

    out_data.rot_mat = rot_mat_;
    out_data.valid = true;
}

Mat3 Stabilizer::getRotationMatrix(const Vec3& a) {
    const float cx = cosf(a.x);   // roll
    const float sx = sinf(a.x);
    const float cy = cosf(a.y);   // pitch
    const float sy = sinf(a.y);
    const float cz = cosf(a.z);   // yaw
    const float sz = sinf(a.z);

    // Standard aerospace ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    return Mat3{
        .data = {{
            {{cz * cy,  cz * sy * sx - sz * cx,  cz * sy * cx + sz * sx}},
            {{sz * cy,  sz * sy * sx + cz * cx,  sz * sy * cx - cz * sx}},
            {{-sy,      cy * sx,                 cy * cx}}
        }}
    };
}