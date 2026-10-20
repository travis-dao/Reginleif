#include "leg_controller.h"
#include "servo.h"

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

void LegController::update(float dt_s, const SharedData& shared_data) {
    bool grounded = true;

    for (int i = 0; i < 4; i++) {
        if (!legs_[i]->isGrounded()) {
            grounded = false;
        }
    }

    this->grounded = grounded;

    // if (this->grounded) {
    //     this->current_airborne_leg = Movement::next_leg_gait[this->current_airborne_leg + 1];
    //     this->legs_[this->current_airborne_leg]->setState(SWING);
    // }

    // update individual legs
    for (int i = 0; i < 4; i++) {
        legs_[i]->update(dt_s, shared_data, *target_angles[i]);
    }

    // write target angles to burst write buffer
    for (int i = 0; i < 4; ++i) {
        target_angles_buffer[i * 3 + 0] = target_angles[i]->coxa;
        target_angles_buffer[i * 3 + 1] = target_angles[i]->femur;
        target_angles_buffer[i * 3 + 2] = target_angles[i]->tibia;
    }

    servoSetAngles(servo_driver_, target_angles_buffer, 12);
}