#include <math.h>
#include <algorithm>

#include "leg.h"
#include "base.h"
#include "servo.h"
#include "pid_stabilize.h"

constexpr float FEMUR_LENGTH = 122.64f;
constexpr float TIBIA_LENGTH = 34.66f + 147.03f;
constexpr float BODY_TO_COXA_X_OFFSET = 28.58f;
constexpr float BODY_TO_COXA_Z_OFFSET = -9.72f;

Leg::Leg(const Config& cfg): 
	info(cfg.id),
	pca(cfg.pca)
{
	this->orientation_offset = 0.0f;
	this->pos_state = {
		.base_neutral_pos = Offset::neutral_vector[cfg.id] * Offset::neutral_offset,
		.true_neutral_pos = Offset::neutral_vector[cfg.id] * (Offset::neutral_offset + Offset::body_offset),
		.curr_pos = this->pos_state.base_neutral_pos,
		.target_pos = this->pos_state.base_neutral_pos,
		.last_grounded_pos = this->pos_state.base_neutral_pos,
	};
	this->phase = 0.0f;
	this->leg_state = HOLD;

	moveLeg();
}

Theta3 Leg::invertAngles(const Theta3& angles) {
	return Theta3 {
		180.0f - angles.coxa,
		180.0f - angles.femur,
		180.0f - angles.tibia
	};
}

Theta3 Leg::clampToLimits(const Theta3& angles) {
	const LegServoLimits& lim = ServoConfig::limits[this->info.id];

	return Theta3 {
		lim.coxa.clamp(angles.coxa),
		lim.femur.clamp(angles.femur),
		lim.tibia.clamp(angles.tibia)
	};
}

Theta3 Leg::IK(Vec3 target_foot_pos) {
	Theta3 new_angles;

	// dis to target pos on x-y plane
	float d = sqrt(target_foot_pos.x * target_foot_pos.x + target_foot_pos.y * target_foot_pos.y);

	// adjust dis for offset to where coxa servo connects femur servo
	float r = d - BODY_TO_COXA_X_OFFSET;

	// dis to target pos on x-z plane, basically dis from femur servo to tip of tibia
	target_foot_pos.z += BODY_TO_COXA_Z_OFFSET;
	float c = sqrt(target_foot_pos.z * target_foot_pos.z + r * r);

	float c_squared = c * c;
	float a_squared = FEMUR_LENGTH * FEMUR_LENGTH;
	float b_squared = TIBIA_LENGTH * TIBIA_LENGTH;

	// calculate femur servo
	float cos2 = std::clamp(
		(a_squared + c_squared - b_squared) / (2 * FEMUR_LENGTH * c), 
		-1.0f, 
		1.0f
	);

	// calculate tibia servo
	float cos3 = std::clamp(
		(a_squared + b_squared - c_squared) / (2 * FEMUR_LENGTH * TIBIA_LENGTH), 
		-1.0f, 
		1.0f
	);

	// update output
	new_angles.coxa = atan2(target_foot_pos.y, target_foot_pos.x) * 180.0f / M_PI; // coxa
	new_angles.femur = atan2(r, -target_foot_pos.z) * 180.0f / M_PI + acos(cos2) * 180.0f / M_PI; // femur
	new_angles.tibia = acos(cos3) * 180.0f / M_PI; // tibia

	return new_angles;
}

void Leg::moveLeg() {
	// flip y pos
	Vec3 adjusted_target_pos = this->pos_state.target_pos;
	adjusted_target_pos.y *= this->info.is_right_leg ? -1.0f : 1.0f;

	// check to flip angles for reverse mounted servos (leg side)
	Theta3 target_angles = IK(adjusted_target_pos);

	if (!this->info.is_right_leg) {
		target_angles = invertAngles(target_angles);
	}

	// clamp servos to calibrated limits
	target_angles = clampToLimits(target_angles);

	// move servos
	// servoSetAngle(this->pca, this->info.id * 3, target_angles.coxa);
	// servoSetAngle(this->pca, this->info.id * 3 + 1, target_angles.femur);
	// servoSetAngle(this->pca, this->info.id * 3 + 2, target_angles.tibia);
	
	printf("Leg %d |	coxa: %f, femur: %f, tibia %f\n", this->info.id, target_angles.coxa, target_angles.femur, target_angles.tibia);

	// update members
	this->angles = target_angles;
}

void Leg::applyStabilization(const float dt_s, const Mat3& rot_mat) {
	// apply rot matrix to pos
	if (dt_s > 0.0f && this->leg_state != LegState::SWING) {
		this->pos_state.target_pos = rot_mat * this->pos_state.target_pos;
	}
}

void Leg::updateStance(const float dt_s, const SharedData& shared_data) {
	// reset phase for swing state
	this->phase = 0.0f;

	// move leg backwards at body's current speed
	this->pos_state.target_pos.x += shared_data.current_speed * dt_s * -1.0f;
}

void Leg::updateSwing(const float dt_s, const float step_length, const float step_height) {
	// swing done -> move to stance phase
	if (this->phase >= 1.0f) {
		setState(STANCE);
		return;
	}

	// x = velocity-based
	float velocity_x = (M_PI * step_length / (2.0f * Movement::SWING_DURATION_S)) * sin(M_PI * this->phase);
	this->pos_state.target_pos.x += velocity_x * dt_s;

	// z = position-based, add on top of whatever update_orientation() just set
	this->pos_state.target_pos.z = this->pos_state.base_neutral_pos.z + step_height * sin(M_PI * this->phase);

	// advance phase
	this->phase += dt_s / Movement::SWING_DURATION_S;
	if (this->phase > 1.0f) this->phase = 1.0f;
}

void Leg::update(const float dt_s, const SharedData& shared_data, Theta3& out_target_angles) {
	applyStabilization(dt_s, shared_data.rot_mat);

	// 2. update target pos based on state
	if (this->leg_state == SWING) {
		// calc step length from where leg lifts
		float target_pos_x = this->pos_state.base_neutral_pos.x + (Movement::MAX_STEP_LENGTH_MM / 2.0f);
		float adjusted_step_length = target_pos_x - this->pos_state.last_grounded_pos.x;

		updateSwing(dt_s, adjusted_step_length, Movement::STEP_HEIGHT_MM);
	} else if (this->leg_state == STANCE) {
		updateStance(dt_s, shared_data);
	} else {
		// this->pos_state.target_pos = this->pos_state.base_neutral_pos; // TODO: CHANGE LATER
	}

	// 3. while grounded, update pos to calculate proper step length
	if (isGrounded()) {
		this->pos_state.last_grounded_pos = this->pos_state.target_pos;
	}

	// 4. move servos based on target pos
	moveLeg();

	// update variables
	this->pos_state.curr_pos = this->pos_state.target_pos;
	out_target_angles = this->angles;
}

void Leg::balance(const Vec3& angles) {
	// printf("balancing\n");
	if (this->leg_state == SWING) return;

	// 3d plane method
	float A = -tan(angles.y * M_PI / 180.0f);
	float B = tan(angles.x * M_PI / 180.0f);

	float offset = -(A * this->pos_state.true_neutral_pos.x + B * this->pos_state.true_neutral_pos.y);

	// difference between target z pos and current z pos
	float offset_delta = fabsf((Offset::neutral_offset.z + offset) - this->pos_state.curr_pos.z);

	// only move leg if change is substantial
	if (offset_delta > 0.0f) {
		this->pos_state.target_pos = this->pos_state.base_neutral_pos + Vec3 { 0.0f, 0.0f, offset };
	}
}