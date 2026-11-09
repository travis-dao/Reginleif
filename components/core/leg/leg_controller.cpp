#include "leg_controller.h"
#include "servo.h"
#include "frame.h"

const float NUM_SERVOS = 12;

LegController::LegController() {
    for (int i = 0; i < 4; i++) {
        legs_[i] = nullptr;
    }

    grounded_ = true;
    swing_leg_ = -1;
    state_ = MoveState::REST;
}

void LegController::init(i2c_master_bus_handle_t bus) {
    // Initialize servo driver and connect to master I2C bus
    Pca9685::Config cfg{};
    cfg.bus = bus;
    cfg.pwm_freq_hz = ServoConfig::SERVO_FREQ;
    ESP_ERROR_CHECK(servo_driver_.init(cfg));

    printf("Initialized Servo Driver\n");
    vTaskDelay(pdMS_TO_TICKS(1000));


    // Instantiate legs
    for (int i = 0; i < 4; i++) {
        legs_[i] = new Leg({i});
    }

    printf("Initialized LegController\n");
    vTaskDelay(pdMS_TO_TICKS(500));
}

void LegController::update(float dt_s, const ImuData& imu_data) {
    // gait sequencing
    bool all_grounded_ = true;
    for (int i = 0; i < 4; i++) {
        if (!legs_[i]->grounded()) { 
            all_grounded_ = false;
            break;
        }
    }
    grounded_ = all_grounded_;

    // only start next swing sequence if all legs are grounded_
    if (grounded_) {
        if (state_ == MoveState::REST) {
            startWalk();
        }

        int next = Movement::next_leg_gait[swing_leg_ + 1];
        swing_leg_ = next;
        legs_[next]->startSwing(legs_[next]->nominalLandingPos());
    }

    // leg movement calculations
    for (int i = 0; i < 4; i++) {
        legs_[i]->update(dt_s, target_angles_[i], imu_data);

        // update servos (burst write)
        target_angles_buffer_[i * 3 + 0] = target_angles_[i].coxa;
        target_angles_buffer_[i * 3 + 1] = target_angles_[i].femur;
        target_angles_buffer_[i * 3 + 2] = target_angles_[i].tibia;
    }
    servoSetAngles(servo_driver_, target_angles_buffer_, NUM_SERVOS);
}

void LegController::startWalk() {
    swing_leg_ = -1;
    state_ = WALK;

    for (auto leg : legs_) {
        leg->startStance(leg->currFootPos());
    }
}

// current foot pos, body frame
Array<Vec3, 4> LegController::footPositionsBody() const {
	Array<Vec3, 4> out;
	for (int i = 0; i < 4; i++) {
        out[i] = legToBodyFrame(i, legs_[i]->currFootPos());
    }
	return out;
}

Array<Vec3, 4> LegController::landingTargetsBody() const {
	Array<Vec3, 4> out;
	for (int i = 0; i < 4; i++) {
		out[i] = legToBodyFrame(i, legs_[i]->nominalLandingPos());
	}
	return out;
}

void LegController::applyMpcCorrections(Array<Vec3, 4> offsets_lf) {
    for (int i = 0; i < 4; i++) {
        legs_[i]->setMpcOffset(offsets_lf[i]);
    }
}