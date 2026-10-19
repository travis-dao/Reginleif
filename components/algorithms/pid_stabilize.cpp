#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "pid_stabilize.h"

Stabilizer::Stabilizer(const Config& config) {
    this->K_p = config.K_p;
    this->K_i = config.K_i;
    this->K_d = config.K_d;

    this->integral_sum = Vec3 { 0.0f, 0.0f, 0.0f };
    this->last_error = Vec3 { 0.0f, 0.0f, 0.0f };

    this->rot_mat = Mat3 {{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    }};
}

esp_err_t Stabilizer::init(i2c_master_bus_handle_t bus) {
    // Initialize IMU and connect to I2C bus
    esp_log_level_set("mpu6050", ESP_LOG_ERROR);

    mpu6050::Mpu6050::Config cfg;
    cfg.bus = bus;

    esp_err_t err = this->imu_.init(cfg);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Initialized IMU\n");

    // Calibrate IMU
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("Calibrating IMU, do not move\n");

    err = this->imu_.calibrateGyro(200);
    if (err != ESP_OK) {
        return err;
    }

    err = this->imu_.calibrateLevel(200);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("IMU calibration complete\n");

    return ESP_OK;
}

esp_err_t Stabilizer::updateImu() {
    // try to update imu with new measurements
    if (this->imu_.getAngles(this->angles) != ESP_OK) {
        printf("Failed to read IMU\n");
        return ESP_FAIL;
    }

    // printf("roll: %.2f | pitch: %.2f\n", current_orientation.x,
    // current_orientation.y);
    return ESP_OK;
}

void Stabilizer::updatePID(const float dt_s, const Vec3& current_angles) {

    // error
    Vec3 error = target_angles - current_angles;

    // pid stuff
    Vec3 proportional_term = error * K_p;

    this->integral_sum = this->integral_sum + (error * dt_s);

    Vec3 integral_term = this->integral_sum * K_i;

    Vec3 derivative_term = (error - this->last_error) / dt_s * K_d;

    // add the P, I, D terms
    Vec3 compensation_angles_deg = proportional_term + integral_term + derivative_term;

    Vec3 compensation_angles = compensation_angles_deg * (M_PI / 180);

    this->last_error = error;

    this->rot_mat = getRotationMatrix(compensation_angles);
}

esp_err_t Stabilizer::update(const float dt_s, SharedData& out_data) {
    // update imu
    if (this->updateImu() != ESP_OK) {
        printf("Failed to update IMU\n");
        return ESP_FAIL;
    }

    // update pid
    this->updatePID(dt_s, this->getAngles());

    // update output data
    out_data.rot_mat = this->rot_mat;
    return ESP_OK;
}

Mat3 Stabilizer::getRotationMatrix(const Vec3& compensation_angles) {
    Mat3 rotation_matrix_x {{
        {1, 								0, 							0},
        {0, cos(compensation_angles.x), -sin(compensation_angles.x)},
        {0, sin(compensation_angles.x), cos(compensation_angles.x)}
    }};
    Mat3 rotation_matrix_y {{
        {cos(compensation_angles.y),  0, sin(compensation_angles.y)},
        {0, 								1, 							 0},
        {-sin(compensation_angles.y), 0, cos(compensation_angles.y)}
    }};
    Mat3 rotation_matrix_z {{
        {cos(compensation_angles.z), -sin(compensation_angles.z), 0},
        {sin(compensation_angles.z), cos(compensation_angles.z),  0},
        {0, 							   0, 							 1}
    }};

    Mat3 rotation_matrix = rotation_matrix_x * rotation_matrix_y * rotation_matrix_z;

    return rotation_matrix;
}