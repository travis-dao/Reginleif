#include "leg_controller.h"
#include "servo.h"
#include <algorithm>

LegController::LegController() {
    for (int i = 0; i < 4; i++) {
        legs_[i] = nullptr;
    }

    grounded = true;
    this->current_airborne_leg = -1;
}

esp_err_t LegController::init(i2c_master_bus_handle_t bus) {
    // Initialize servo driver and connect to master I2C bus
    Pca9685::Config cfg{};
    cfg.bus = bus;
    cfg.pwm_freq_hz = ServoConfig::SERVO_FREQ;
    esp_err_t e = this->servo_driver_.init(cfg);

    if (e != ESP_OK) {
        printf("Failed to initialize PCA9685: %s\n", esp_err_to_name(e));
        return e;
    }

    printf("Initialized Servo Driver\n");
    vTaskDelay(pdMS_TO_TICKS(1000));


    // Instantiate legs
    for (int i = 0; i < 4; i++) {
        this->legs_[i] = new Leg({
            i,
            this->servo_driver_,
        });
    }

    printf("Initialized LegController\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    return ESP_OK;
}

static float approach(float cur, float goal, float max_step) {
    return cur + std::clamp(goal - cur, -max_step, max_step);
}

void LegController::update(float dt_s, const SharedData& shared_data) {
    // ---- 1. gait sequencing
    bool all_grounded = true;
    for (int i = 0; i < 4; i++) {
        if (!legs_[i]->isGrounded()) { 
            all_grounded = false;
            break;
        }
    }
    this->grounded = all_grounded;

    if (this->grounded) {
        int next = Movement::next_leg_gait[this->current_airborne_leg + 1];
        this->current_airborne_leg = next;
        legs_[next]->setState(SWING);
    }

    int lifting = this->current_airborne_leg;
    if (lifting >= 0.0f && legs_[lifting]->getSwingPhase() > 0.7f) {
        lifting = Movement::next_leg_gait[lifting + 1];
    }


    for (int i = 0; i < 4; i++) {
        legs_[i]->update(dt_s, shared_data, *target_angles[i]);
    }


    // burst write
    for (int i = 0; i < 4; ++i) {
        target_angles_buffer[i * 3 + 0] = target_angles[i]->coxa;
        target_angles_buffer[i * 3 + 1] = target_angles[i]->femur;
        target_angles_buffer[i * 3 + 2] = target_angles[i]->tibia;
    }
    servoSetAngles(servo_driver_, target_angles_buffer, 12);
}