#include "esp_log.h"
#include "stabilizer.h"


static constexpr float k_max_comp_deg = 15.0f;
static constexpr float k_integral_max = 10.0f;

Stabilizer::Stabilizer(const Config& config)
    : target_angles_deg{0.0f, 0.0f, 0.0f},
      gyro_dps{0.0f, 0.0f, 0.0f},
      K_p(config.K_p),
      K_i(config.K_i),
      K_d(config.K_d),
      integral_sum{0.0f, 0.0f, 0.0f}
{
    rot_mat = getRotationMatrix(Vec3{0.0f, 0.0f, 0.0f});   // identity until the first update
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

    ESP_ERROR_CHECK(this->imu_.init(cfg));

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Initialized IMU\n");

    // Calibrate IMU
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("Calibrating IMU, do not move\n");

    ESP_ERROR_CHECK(this->imu_.calibrate(200));

    printf("IMU calibration complete\n");
}

void Stabilizer::updateImu() {
    ESP_ERROR_CHECK(this->imu_.getAngles(this->angles_deg));

    Vec3 g = this->imu_.lastGyro();

    // Assumes lastGyro() reports degrees per second.
    this->gyro_dps = Vec3 { g.x, -g.y, 0.0f };
}

void Stabilizer::updatePID(float dt_s, const Vec3& angles_in_deg, const Vec3& gyro_dps) {
    if (dt_s <= 0.0f) return;

    Vec3 error = this->target_angles_deg - angles_in_deg;
    error.z = 0.0f;

    Vec3 d_term = gyro_dps * (-1.0f) * this->K_d;

    // Apply conditional anti-windup to the roll and pitch integrators.
    // Keep the integral from growing when doing so would worsen saturation.
    auto updateIntegralAxis = [&](float error_axis, float d_axis, float kp, float ki, float& integral_axis) {
        if (ki == 0.0f) return;

        const float proposed_integral = std::clamp(
            integral_axis + error_axis * dt_s,
            -k_integral_max,
            k_integral_max
        );

        const float proposed_output =
            kp * error_axis + ki * proposed_integral + d_axis;

        const bool drives_further_into_saturation =
            (proposed_output > k_max_comp_deg && ki * error_axis > 0.0f) ||
            (proposed_output < -k_max_comp_deg && ki * error_axis < 0.0f);

        if (!drives_further_into_saturation) {
            integral_axis = proposed_integral;
        }
    };

    updateIntegralAxis(error.x, d_term.x, this->K_p, this->K_i, this->integral_sum.x);
    updateIntegralAxis(error.y, d_term.y, this->K_p, this->K_i, this->integral_sum.y);
    this->integral_sum.z = 0.0f;

    Vec3 comp_deg = error * this->K_p + this->integral_sum * this->K_i + d_term;

    comp_deg.x = std::clamp(comp_deg.x, -k_max_comp_deg, k_max_comp_deg);
    comp_deg.y = std::clamp(comp_deg.y, -k_max_comp_deg, k_max_comp_deg);
    comp_deg.z = 0.0f;

    this->rot_mat = getRotationMatrix(comp_deg * k_deg_to_rad);
}

void Stabilizer::update(const float dt_s, ImuData& out_data) {
    updateImu();

    updatePID(dt_s, getAngles(), getGyro());

    out_data.rot_mat = this->rot_mat;
    out_data.valid = true;
}

Mat3 Stabilizer::getRotationMatrix(const Vec3& a) {
    const float cx = cosf(a.x);
    const float sx = sinf(a.x);
    const float cy = cosf(a.y);
    const float sy = sinf(a.y);

    // R = Rx * Ry (roll, then pitch; no yaw).
    return Mat3 {{
        { cy,      0.0f,  sy      },
        { sx * sy, cx,    -sx * cy },
        { -cx * sy,sx,    cx * cy }
    }};
}



// esp_err_t Stabilizer::update(const float dt_s, SharedData& out) {
//     esp_err_t err_angle = this->imu_.getAngles(this->angles);
//     if (err_angle != OK) return err_angle;

//     esp_err_t err_gyro = this->imu_.readGyro(this->gyro);
//     if (err_gyro != OK) return err_gyro;

//     out.angles_rad = Vec3 {this->angles.roll, this->angles.pitch, this->angles.yaw} * k_deg_to_rad;
//     out.rpy_rad  = Vec3 {angles.roll, angles.pitch, angles.yaw} * k_deg_to_rad;
//     out.gyro_rad_s = gyro * k_deg_to_rad;   // ADAPT: ISM330 reports dps; convert and remove bias
//     out.valid = true;

//     return ESP_OK;
// }