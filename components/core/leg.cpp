#include <math.h>
#include <algorithm>

#include "leg.h"
#include "config.h"
#include "base.h"
#include "servo.h"
#include "pid_stabilize.h"

Leg::Leg(const Config& cfg): 
	info(cfg.id),
	base_(cfg.base_),
	pca(cfg.pca),
	stabilizer_(cfg.stabilizer_)
{
	this->orientation_offset = 0.0f;
	this->pos_state = {
		.base_neutral_pos = Configs::Offset::neutral_vector[cfg.id] * Configs::Offset::neutral_offset,
		.true_neutral_pos = Configs::Offset::neutral_vector[cfg.id] * (Configs::Offset::neutral_offset + Configs::Offset::body_offset),
		.curr_pos = this->pos_state.base_neutral_pos,
		.target_pos = this->pos_state.base_neutral_pos,
		.last_grounded_pos = this->pos_state.base_neutral_pos,
	};
	this->phase = 0.0f;
	this->leg_state = HOLD;

	move_leg();
}

Theta3 Leg::invert_angles(const Theta3& angles) {
	return Theta3 {
		180.0f - angles.coxa,
		180.0f - angles.femur,
		180.0f - angles.tibia
	};
}

Theta3 Leg::clamp_to_limits(const Theta3& angles) const {
	const LegServoLimits& lim = ServoConfig::limits[this->info.id];

	return Theta3 {
		lim.coxa.clamp(angles.coxa),
		lim.femur.clamp(angles.femur),
		lim.tibia.clamp(angles.tibia)
	};
}

Theta3 Leg::ik(Vec3 target_foot_pos) {
	Theta3 new_angles;

	// dis to target pos on x-y plane
	float d = sqrt(target_foot_pos.x * target_foot_pos.x + target_foot_pos.y * target_foot_pos.y);

	// adjust dis for offset to where coxa servo connects femur servo
	float r = d - Configs::Leg::body_to_coxa_x_offset;

	// dis to target pos on x-z plane, basically dis from femur servo to tip of tibia
	target_foot_pos.z += Configs::Leg::body_to_coxa_z_offset;
	float c = sqrt(target_foot_pos.z * target_foot_pos.z + r * r);

	float c_squared = c * c;
	float a_squared = Configs::Leg::femur_length * Configs::Leg::femur_length;
	float b_squared = Configs::Leg::tibia_length * Configs::Leg::tibia_length;

	// calculate femur servo
	float cos2 = std::clamp(
		(a_squared + c_squared - b_squared) / (2 * Configs::Leg::femur_length * c), 
		-1.0f, 
		1.0f
	);

	// calculate tibia servo
	float cos3 = std::clamp(
		(a_squared + b_squared - c_squared) / (2 * Configs::Leg::femur_length * Configs::Leg::tibia_length), 
		-1.0f, 
		1.0f
	);

	// update output
	new_angles.coxa = atan2(target_foot_pos.y, target_foot_pos.x) * 180.0f / M_PI; // coxa
	new_angles.femur = atan2(r, -target_foot_pos.z) * 180.0f / M_PI + acos(cos2) * 180.0f / M_PI; // femur
	new_angles.tibia = acos(cos3) * 180.0f / M_PI; // tibia

	return new_angles;
}

void Leg::move_leg() {
	Vec3 adjusted_target_pos = this->pos_state.target_pos;
	adjusted_target_pos.y *= this->info.is_right_leg ? -1.0f : 1.0f;

	// check to flip angles for reverse mounted servos (leg side)
	Theta3 target_angles = ik(adjusted_target_pos);

	if (this->info.is_right_leg) {
		invert_angles(target_angles);
	}

	// clamp servos to calibrated limits
	clamp_to_limits(target_angles);

	// move servos
	servo_set_angle(this->pca, this->info.id * 3, target_angles.coxa);
	servo_set_angle(this->pca, this->info.id * 3 + 1, target_angles.femur);
	servo_set_angle(this->pca, this->info.id * 3 + 2, target_angles.tibia);
	
	// printf("Leg %d |	coxa: %f, femur: %f, tibia %f\n", this->info.id, target_angles.coxa, target_angles.femur, target_angles.tibia);

	// update members
	this->angles = target_angles;
	this->pos_state.curr_pos = this->pos_state.target_pos;
}

void Leg::update_orientation() {
	Vec3 target_orientation = this->base_.get_target_orientation();

	float A = -tan(target_orientation.y * M_PI / 180.0f);
	float B = tan(target_orientation.x * M_PI / 180.0f);

	this->orientation_offset = -(A * this->pos_state.true_neutral_pos.x + B * this->pos_state.true_neutral_pos.y);

	// if (this->state == SWING) {
	// 	this->orientation_offset = 0.0f;
	// }

	this->pos_state.target_pos.z = this->pos_state.base_neutral_pos.z + this->orientation_offset;
}

void Leg::apply_pid_stabilization() {
	// apply rot matrix to pos
	if (this->base_.get_dt_s() > 0.0f && this->leg_state != LegState::SWING) {
		this->pos_state.target_pos = this->stabilizer_.rot_mat * this->pos_state.target_pos;
	}
}

void Leg::update_stance() {
	// reset phase for swing state
	this->phase = 0.0f;

	// move leg backwards at body's current speed
	this->pos_state.target_pos.x += this->base_.get_speed() * this->base_.get_dt_s() * -1.0f;
}

void Leg::update_swing(const float step_length, const float step_height) {
	// swing done -> move to stance phase
	if (this->phase >= 1.0f) {
		update_state(STANCE);
		return;
	}

	// x = velocity-based
	float velocity_x = (M_PI * step_length / (2.0f * Configs::Movement::SWING_DURATION_S)) * sin(M_PI * this->phase);
	this->pos_state.target_pos.x += velocity_x * this->base_.get_dt_s();

	// z = position-based, add on top of whatever update_orientation() just set
	this->pos_state.target_pos.z = this->pos_state.base_neutral_pos.z + step_height * sin(M_PI * this->phase);

	// advance phase
	this->phase += this->base_.get_dt_s() / Configs::Movement::SWING_DURATION_S;
	if (this->phase > 1.0f) this->phase = 1.0f;
}

void Leg::update() {
	const Vec3 imu_angles = this->base_.get_imu_angles();

	// 1. update orientation offset and target_pos.z based on body's orientation
	update_orientation();
	apply_pid_stabilization();

	// 2. update target pos based on state
	if (this->leg_state == SWING) {
		// calc step length from where leg lifts
		float target_pos_x = this->pos_state.base_neutral_pos.x + (Configs::Movement::MAX_STEP_LENGTH_MM / 2.0f);
		float adjusted_step_length = target_pos_x - this->pos_state.last_grounded_pos.x;

		update_swing(adjusted_step_length, Configs::Movement::STEP_HEIGHT_MM);
	} else if (this->leg_state == STANCE) {
		update_stance();
	} else {
		// this->pos_state.target_pos = this->pos_state.base_neutral_pos; // TODO: CHANGE LATER
	}

	// balance(imu_angles);

	// 3. while grounded, update pos to calculate proper step length
	if (is_grounded()) {
		this->pos_state.last_grounded_pos = this->pos_state.target_pos;
	}

	// 4. move servos based on target pos
	move_leg();
}

void Leg::balance(const Vec3& angles) {
	// printf("balancing\n");
	if (this->leg_state == SWING) return;

	// 3d plane method
	float A = -tan(angles.y * M_PI / 180.0f);
	float B = tan(angles.x * M_PI / 180.0f);

	float offset = -(A * this->pos_state.true_neutral_pos.x + B * this->pos_state.true_neutral_pos.y);

	// difference between target z pos and current z pos
	float offset_delta = fabsf((Configs::Offset::neutral_offset.z + offset) - this->pos_state.curr_pos.z);

	// only move leg if change is substantial
	if (offset_delta > 0.0f) {
		this->pos_state.target_pos = this->pos_state.base_neutral_pos + Vec3 { 0.0f, 0.0f, offset };
	}
}