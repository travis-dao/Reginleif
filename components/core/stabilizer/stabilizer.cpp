#include "esp_log.h"
#include "stabilizer.h"
#include <algorithm>


static constexpr float MAX_COMP_DEG = 15.0f;
static constexpr float INTEGRAL_MAX = 10.0f;

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
    ESP_ERROR_CHECK(imu_.getAngles(angles_deg_));

    Vec3 g = imu_.lastGyro();

    // Assumes lastGyro() reports degrees per second.
    gyro_dps_ = Vec3 { g.x, -g.y, 0.0f };
}

void Stabilizer::updatePID(float dt_s, const Vec3& angles_in_deg, const Vec3& gyro_dps) {
    if (dt_s <= 0.0f) return;

    Vec3 error = target_angles_deg_ - angles_in_deg;
    error.z = 0.0f;

    Vec3 d_term = gyro_dps * (-1.0f) * K_d_;

    // Apply conditional anti-windup to the roll and pitch integrators.
    // Keep the integral from growing when doing so would worsen saturation.
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
    integral_sum_.z = 0.0f;

    Vec3 comp_deg = error * K_p_ + integral_sum_ * K_i_ + d_term;

    comp_deg.x = std::clamp(comp_deg.x, -MAX_COMP_DEG, MAX_COMP_DEG);
    comp_deg.y = std::clamp(comp_deg.y, -MAX_COMP_DEG, MAX_COMP_DEG);
    comp_deg.z = 0.0f;

    rot_mat_ = getRotationMatrix(comp_deg * DEG_TO_RAD);
}

void Stabilizer::update(const float dt_s, ImuData& out_data) {
    updateImu();

    updatePID(dt_s, angles(), gyro());

    out_data.rot_mat = rot_mat_;
    out_data.valid = true;
}

Mat3 Stabilizer::getRotationMatrix(const Vec3& a) {
    const float cx = cosf(a.x);
    const float sx = sinf(a.x);
    const float cy = cosf(a.y);
    const float sy = sinf(a.y);

    // R = Rx * Ry (roll, then pitch; no yaw).
    return Mat3{
        .data = {{
            {{cy,       0.0f,  sy}},
            {{sx * sy,  cx,   -sx * cy}},
            {{-cx * sy, sx,    cx * cy}}
        }}
    };
}



// esp_err_t Stabilizer::update(const float dt_s, SharedData& out) {
//     esp_err_t err_angle = imu_.angles(angles);
//     if (err_angle != OK) return err_angle;

//     esp_err_t err_gyro = imu_.readGyro(gyro);
//     if (err_gyro != OK) return err_gyro;

//     out.angles_rad = Vec3 {angles.roll, angles.pitch, angles.yaw} * DEG_TO_RAD;
//     out.rpy_rad  = Vec3 {angles.roll, angles.pitch, angles.yaw} * DEG_TO_RAD;
//     out.gyro_rad_s = gyro * DEG_TO_RAD;   // ADAPT: ISM330 reports dps; convert and remove bias
//     out.valid = true;

//     return ESP_OK;
// }