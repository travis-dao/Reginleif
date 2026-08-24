#include <math.h>
#include <algorithm>

#include <mpu6050.h>

#include "leg.h"
#include "helpers.h"
#include "base.h"

Info::Info(int id) {
	this->id = id;
	this->is_front_leg = id == 0 || id == 1;
	this->is_right_leg = id == 1 || id == 2;
}

Leg::Leg(int id) : info(id) {
	this->orientation_offset = 0.0f;
	this->base_neutral_pos = NeutralConfig::neutral_vector[id] * NeutralConfig::neutral_offset;
	this->true_neutral_pos = NeutralConfig::neutral_vector[id] * (NeutralConfig::neutral_offset + body_offset);
	this->last_grounded_pos = this->base_neutral_pos;
	this->curr_pos = this->base_neutral_pos;
	this->target_pos = this->base_neutral_pos;
	this->phase = 0.0f;
	this->state = HOLD;

	// this->base_neutral_pos.println();

	move_leg();
}

// for left side servos, had to mount in reverse
Theta3 Leg::get_inverted_angles(Theta3 out_angles) {
	out_angles.theta1 = 180.0f - out_angles.theta1;
	out_angles.theta2 = 180.0f - out_angles.theta2;
	out_angles.theta3 = 180.0f - out_angles.theta3;

	return out_angles;
}

Theta3 Leg::ik(const float x, const float y, const float z) {
	Theta3 new_angles;

	float d = sqrt(x * x + y * y);
	float r = d - l;
	float c = sqrt(z * z + r * r);

	float cos2 = std::clamp((a * a + c * c - b * b) / (2 * a * c), -1.0f, 1.0f);
	float cos3 = std::clamp((a * a + b * b - c * c) / (2 * a * b), -1.0f, 1.0f);

	new_angles.theta1 = atan2(y, x) * 180.0f / M_PI; // coxa
	new_angles.theta2 = atan2(r, -z) * 180.0f / M_PI + acos(cos2) * 180.0f / M_PI; // femur
	new_angles.theta3 = 180.0 - acos(cos3) * 180.0f / M_PI; // tibia

	return new_angles;
}

void Leg::update_state(const LegState state) {
  	this->state = state;
}

// move 1 leg
void Leg::move_leg() {
	Vec3 adjusted = this->target_pos;

	adjusted.y *= this->info.is_right_leg ? -1 : 1;

	Theta3 target_angles = ik(adjusted.x, adjusted.y, adjusted.z);

	// flip angles cuz servos on left leg flipped
	target_angles = this->info.is_right_leg ? target_angles : get_inverted_angles(target_angles);
	
	printf("Leg %d |	theta1: %f, theta2: %f, theta3 %f\n", this->info.id, target_angles.theta1, target_angles.theta2, target_angles.theta3);

	// move servos
	esp_err_t err;
	err = pca9685_set_pwm_value(&Base::pca, this->info.id * 3,     angle_to_pulse(target_angles.theta1));
	if (err != ESP_OK) printf("Failed to set servo PWM (theta1)\n");

	err = pca9685_set_pwm_value(&Base::pca, this->info.id * 3 + 1, angle_to_pulse(target_angles.theta2));
	if (err != ESP_OK) printf("Failed to set servo PWM (theta2)\n");

	err = pca9685_set_pwm_value(&Base::pca, this->info.id * 3 + 2, angle_to_pulse(target_angles.theta3));
	if (err != ESP_OK) printf("Failed to set servo PWM (theta3)\n");

	// update members
	this->angles = target_angles;
	this->curr_pos = this->target_pos;
}

// TODO -> migrate PID controller and tune params
void Leg::update_orientation() {
	Vec3 target_orientation = base.get_target_orientation();

	float A = -tan(target_orientation.y * M_PI / 180.0f);
	float B = tan(target_orientation.x * M_PI / 180.0f);

	this->orientation_offset = -(A * true_neutral_pos.x + B * true_neutral_pos.y);

	// if (this->state == SWING) {
	// 	this->orientation_offset = 0.0f;
	// }

	this->target_pos.z = base_neutral_pos.z + this->orientation_offset;
}

void Leg::update_stance() {
	this->phase = 0.0f;

	target_pos.x += base.get_speed() * base.get_dt_s() * -1;
}

void Leg::update_swing(const float step_length, const float step_height) {
	if (this->phase >= 1.0f) {
		update_state(STANCE);
		return;
	}

	// x = velocity-based
	float velocity_x = (M_PI * step_length / (2.0f * MovementConfig::SWING_DURATION_S)) * sin(M_PI * this->phase);
	this->target_pos.x += velocity_x * base.get_dt_s();

	// z = position-based, add on top of whatever update_orientation() just set
	this->target_pos.z = this->base_neutral_pos.z + step_height * sin(M_PI * this->phase);

	// advance phase
	this->phase += base.get_dt_s() / MovementConfig::SWING_DURATION_S;
	if (this->phase > 1.0f) this->phase = 1.0f;
}

void Leg::update() {
	update_orientation();

	// update target pos
	if (this->state == SWING) {
		// calc step length from where leg lifts
		float target_pos_x = this->base_neutral_pos.x + (MovementConfig::MAX_STEP_LENGTH_MM / 2);
		float step_length = target_pos_x - last_grounded_pos.x;

		update_swing(step_length, MovementConfig::STEP_HEIGHT_MM);
	} else if (this->state == STANCE) {
		update_stance();
	} else {
		this->target_pos = this->base_neutral_pos; // TEMP, CHANGE LATER
	}

	if (is_grounded()) {
		this->last_grounded_pos = this->target_pos;
	}

	// move servos
	move_leg();
}