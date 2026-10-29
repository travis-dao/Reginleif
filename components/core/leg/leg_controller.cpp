#include "leg_controller.h"
#include "servo.h"
#include <algorithm>

const float k_num_servos = 12;

LegController::LegController() {
    for (int i = 0; i < 4; i++) {
        legs_[i] = nullptr;
    }

    grounded = true;
    this->swing_leg = -1;
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
    // gait sequencing
    bool all_grounded = true;
    for (int i = 0; i < 4; i++) {
        if (!legs_[i]->isGrounded()) { 
            all_grounded = false;
            break;
        }
    }
    this->grounded = all_grounded;

    if (this->grounded) {
        int next = Movement::next_leg_gait[this->swing_leg + 1];
        this->swing_leg = next;
        legs_[next]->setState(SWING);
    }

    // leg movement calculations
    for (int i = 0; i < 4; i++) {
        this->legs_[i]->update(dt_s, shared_data, target_angles[i]);
    }

    // update servos (burst write)
    for (int i = 0; i < 4; ++i) {
        this->target_angles_buffer[i * 3 + 0] = this->target_angles[i].coxa;
        this->target_angles_buffer[i * 3 + 1] = this->target_angles[i].femur;
        this->target_angles_buffer[i * 3 + 2] = this->target_angles[i].tibia;
    }
    servoSetAngles(this->servo_driver_, this->target_angles_buffer, k_num_servos);
}

// Everything here is mm, body frame (x forward, y left, z up, origin at the body center)


// Leg-frame foot position -> true body-frame position
Vec3 legToBodyFrame(int leg, const Vec3& foot_leg_lf) {
	return Offset::leg_base_pos_bf[leg] + foot_leg_lf;
}



// Where each foot is now: the commanded target (FK of the commanded angles; you have no measured
// ones). Includes any correction set through setFootCorrections().
Array<Vec3, 4> LegController::footPositionsBody() const {
	Array<Vec3, 4> out;
	for (int i = 0; i < 4; ++i) {
        out[i] = legToBodyFrame(i, this->legs_[i]->getFootTarget());
    }
	return out;
}

// Where each foot touches down. Only the airborne leg differs from its current position. The next
// leg to lift lands after the MPC horizon ends (horizon = one swing), so it doesn't need one.
Array<Vec3, 4> LegController::landingTargetsBody() const {
	Array<Vec3, 4> out;
	for (int i = 0; i < 4; ++i) {
		const Vec3 foot_pos_lf = 
            (i == this->swing_leg) 
            ? this->legs_[i]->getSwingEndTarget() 
            : this->legs_[i]->getFootTarget();
		out[i] = legToBodyFrame(i, foot_pos_lf);
	}
	return out;
}

// Contact schedule. Your gait lifts one leg at a time, each for k_swing_duration_s, in the order given
// by Movement::next_leg_gait (FL -> BR -> FR -> BL). out[k*4 + leg] is true if that foot is down
// during step k of length dt_step.
void LegController::predictFootContacts(float dt_step, bool* out, int n_steps) const {
    // nothing airborne: all feet down
	if (this->swing_leg < 0) {
		for (int i = 0; i < n_steps * 4; ++i) {
            out[i] = true;
        }
		return;
	}

    // when it lands
	const float first_end = this->legs_[this->swing_leg]->getSwingTimeRemainingS();

	for (int k = 0; k < n_steps; ++k) {
		const float t_mid = (k + 0.5f) * dt_step;

		int airborne = this->swing_leg;
		float end = first_end;

        // walk the swing chain forward in time
		while (t_mid >= end) {
			airborne = Movement::next_leg_gait[airborne + 1]; // table is indexed by leg + 1
			end += Movement::k_swing_duration_s;
		}

		for (int i = 0; i < 4; ++i) {
            out[k * 4 + i] = (i != airborne);
        }
	}
}

// Per-foot corrections from the MPC mapper. Pushed straight into each Leg, which adds the offset to
// its foot target before IK; leg_controller.cpp's update() needs no changes.
void LegController::setFootCorrections(const Array<Vec3, 4> delta_mm) {
	for (int i = 0; i < 4; ++i) {
		this->legs_[i]->setFootOffset(delta_mm[i]);
	}
}