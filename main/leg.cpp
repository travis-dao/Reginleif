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

	move_leg();
}

Theta3 Leg::get_inverted_angles(Theta3 out_angles) {
	out_angles.coxa = 180.0f - out_angles.coxa;
	out_angles.femur = 180.0f - out_angles.femur;
	out_angles.tibia = 180.0f - out_angles.tibia;

	return out_angles;
}

Theta3 Leg::ik(Vec3 target_foot_pos) {
	Theta3 new_angles;

	// dis to target pos on x-y plane
	float d = sqrt(target_foot_pos.x * target_foot_pos.x + target_foot_pos.y * target_foot_pos.y);

	// adjust dis for offset to where coxa servo connects femur servo
	float r = d - LegConfig::body_to_coxa_x_offset;

	// dis to target pos on x-z plane, basically dis from femur servo to tip of tibia
	target_foot_pos.z += LegConfig::body_to_coxa_z_offset;
	float c = sqrt(target_foot_pos.z * target_foot_pos.z + r * r);

	float c_squared = c * c;
	float a_squared = LegConfig::femur_length * LegConfig::femur_length;
	float b_squared = LegConfig::tibia_length * LegConfig::tibia_length;

	// calculate femur servo
	float cos2 = std::clamp(
		(a_squared + c_squared - b_squared) / (2 * LegConfig::femur_length * c), 
		-1.0f, 
		1.0f
	);

	// calculate tibia servo
	float cos3 = std::clamp(
		(a_squared + b_squared - c_squared) / (2 * LegConfig::femur_length * LegConfig::tibia_length), 
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
	Vec3 adjusted_target_pos = this->target_pos;

	adjusted_target_pos.y *= this->info.is_right_leg ? -1 : 1;

	Theta3 target_angles = ik(adjusted_target_pos);

	// flip angles cuz servos on left leg flipped
	target_angles = this->info.is_right_leg ? target_angles : get_inverted_angles(target_angles);
	
	// printf("Leg %d |	coxa: %f, femur: %f, tibia %f\n", this->info.id, target_angles.coxa, target_angles.femur, target_angles.tibia);

	// move servos
	servo_set_angle(Base::pca, this->info.id * 3, target_angles.coxa);
	servo_set_angle(Base::pca, this->info.id * 3 + 1, target_angles.femur);
	servo_set_angle(Base::pca, this->info.id * 3 + 2, target_angles.tibia);

	// update members
	this->angles = target_angles;
	this->curr_pos = this->target_pos;
}

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

void Leg::apply_pid_stabilization() {
	if (base.get_dt_s() > 0 && state != LegState::SWING) {
		this->target_pos = base.get_rot_matrix() * this->target_pos;
	}
}

void Leg::update_stance() {
	// reset phase for swing state
	this->phase = 0.0f;

	// move leg backwards at body's current speed
	target_pos.x += base.get_speed() * base.get_dt_s() * -1;
}

void Leg::update_swing(const float step_length, const float step_height) {
	// swing done -> move to stance phase
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
	const Vec3 imu_angles = base.get_imu_angles();

	// 1. update orientation offset and target_pos.z based on body's orientation
	// update_orientation();
	// apply_pid_stabilization();

	// 2. update target pos based on state
	if (this->state == SWING) {
		// calc step length from where leg lifts
		float target_pos_x = this->base_neutral_pos.x + (MovementConfig::MAX_STEP_LENGTH_MM / 2);
		float adjusted_step_length = target_pos_x - last_grounded_pos.x;

		update_swing(adjusted_step_length, MovementConfig::STEP_HEIGHT_MM);
	} else if (this->state == STANCE) {
		update_stance();
	} else {
		// this->target_pos = this->base_neutral_pos; // TODO: CHANGE LATER
		balance(imu_angles);
	}

	// 3. while grounded, update pos to calculate proper step length
	if (is_grounded()) {
		this->last_grounded_pos = this->target_pos;
	}

	// 4. move servos based on target pos
	move_leg();
}

void Leg::balance(const Vec3& angles) {
	if (state == SWING) return;

	float A = -tan(angles.y * M_PI / 180);
	float B = tan(angles.x * M_PI / 180);

	float offset = -(A * true_neutral_pos.x + B * true_neutral_pos.y);

	// difference between target z pos and current z pos
	float offset_delta = fabsf((NeutralConfig::neutral_offset.z + offset) - curr_pos.z);

	// only move leg if change is substantial
	if (offset_delta > 1) {
		target_pos = base_neutral_pos + Vec3 { 0, 0, offset };
	}
}