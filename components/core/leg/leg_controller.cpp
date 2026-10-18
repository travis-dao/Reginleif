#include "leg_controller.h"
#include "servo.h"

LegController::LegController() {
    for (int i = 0; i < 4; i++) {
        legs_[i] = nullptr;
    }

    grounded = true;
    this->current_airborne_leg = -1;
}

esp_err_t LegController::init(i2c_master_bus_handle_t bus)
{
    this->bus_ = bus;

    // Initialize servo driver and connect to master I2C bus
    pca9685::Pca9685::Config cfg{};
    cfg.bus = bus_;
    cfg.pwm_freq_hz = ServoConfig::SERVO_FREQ;
    esp_err_t e = this->servo_driver_.init(cfg);

    if (e != ESP_OK) {
        printf("Failed to initialize PCA9685: %s\n", esp_err_to_name(e));
        return e;
    }

    printf("Servo driver initialized\n");
    vTaskDelay(pdMS_TO_TICKS(1000));


    // Instantiate legs
    for (int i = 0; i < 4; i++) {
        this->legs_[i] = new Leg({
            i,
            this->servo_driver_,
        });
    }

    printf("LegController initialized\n");

    return ESP_OK;
}

void LegController::update(float dt_s, const SharedData& shared_data) {
    grounded = true;

    for (int i = 0; i < 4; i++) {
        if (!legs_[i]->isGrounded()) {
            grounded = false;
        }
    }

    if (this->grounded) {
        this->current_airborne_leg = Movement::next_leg_gait[this->current_airborne_leg + 1];
        this->legs_[this->current_airborne_leg]->updateState(SWING);
    }

    for (int i = 0; i < 4; i++) {
        legs_[i]->update(dt_s, shared_data);
    }
}