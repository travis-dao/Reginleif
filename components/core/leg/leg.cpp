#include <cmath>
#include <math.h>
#include <algorithm>

#include "leg.h"
#include "base.h"
#include "leg_controller.h"
#include "servo.h"
#include "types.h"

constexpr float k_femur_length_mm = 122.64f;
constexpr float k_tibia_length_mm = 34.66f + 147.03f;
constexpr float k_body_coxa_x_mm = 28.58f;
constexpr float k_body_coxa_z_mm = 0.0f;

extern bool isRightLeg(const int id) {
	return id == 1 || id == 2;
}
extern bool isFrontLeg(const int id) {
	return id == 0 || id == 1;
}


Leg::Leg(const Config& cfg):
	pca(cfg.pca)
{
	Vec3 neutral_foot_pos_lf = Offset::neutral_foot_pos_lf[cfg.id];

	this->id = cfg.id;
	this->pos_state = {
		.foot_curr_pos_lf = neutral_foot_pos_lf,
		.foot_target_pos_lf = neutral_foot_pos_lf,
		.foot_last_grounded_pos_lf = neutral_foot_pos_lf,
		.foot_offset_lf = Vec3 {0.0f, 0.0f, 0.0f}
	};

	this->swing_phase = 0.0f;
	this->leg_state = HOLD;

	moveLeg();
}

Theta3 Leg::invertAngles(const Theta3& angles_deg) {
	return Theta3 {
		180.0f - angles_deg.coxa,
		180.0f - angles_deg.femur,
		180.0f - angles_deg.tibia
	};
}

Theta3 Leg::clampToLimits(const Theta3& angles_deg) {
	const LegServoLimits& lim = ServoConfig::limits[id];

	return Theta3 {
		lim.coxa.clamp(angles_deg.coxa),
		lim.femur.clamp(angles_deg.femur),
		lim.tibia.clamp(angles_deg.tibia)
	};
}

Theta3 Leg::IK(Vec3 target_foot_pos_lf) {
	Theta3 target_angles_deg;

	// dis to target pos on x-y plane
	float d = sqrt(target_foot_pos_lf.x * target_foot_pos_lf.x + target_foot_pos_lf.y * target_foot_pos_lf.y);

	// adjust dis for offset to where coxa servo connects femur servo
	float r = d - k_body_coxa_x_mm;

	// dis to target pos on x-z plane, basically dis from femur servo to tip of tibia
	target_foot_pos_lf.z += k_body_coxa_z_mm;
	float c = sqrt(target_foot_pos_lf.z * target_foot_pos_lf.z + r * r);

	float c_squared = c * c;
	float a_squared = k_femur_length_mm * k_femur_length_mm;
	float b_squared = k_tibia_length_mm * k_tibia_length_mm;

	// calculate femur servo
	float cos2 = std::clamp(
		(a_squared + c_squared - b_squared) / (2 * k_femur_length_mm * c), 
		-1.0f, 
		1.0f
	);

	// calculate tibia servo
	float cos3 = std::clamp(
		(a_squared + b_squared - c_squared) / (2 * k_femur_length_mm * k_tibia_length_mm), 
		-1.0f, 
		1.0f
	);

	// update output
	target_angles_deg.coxa = atan2f(target_foot_pos_lf.y, target_foot_pos_lf.x) * k_rad_to_deg; // coxa
	target_angles_deg.femur = atan2f(r, -target_foot_pos_lf.z) * k_rad_to_deg + acosf(cos2) * k_rad_to_deg; // femur
	target_angles_deg.tibia = acosf(cos3) * k_rad_to_deg; // tibia

	return target_angles_deg;
}

void Leg::moveLeg() {
	// flip y pos
	Vec3 adjusted_target_pos_lf = this->pos_state.foot_target_pos_lf + this->pos_state.foot_offset_lf;
	adjusted_target_pos_lf.y *= isRightLeg(id) ? -1.0f : 1.0f;

	// check to flip angles for reverse mounted servos (leg side)
	Theta3 target_angles_deg = IK(adjusted_target_pos_lf);

	if (!isRightLeg(id)) {
		target_angles_deg = invertAngles(target_angles_deg);
	}

	// clamp servos to calibrated limits
	target_angles_deg = clampToLimits(target_angles_deg);
	
	// printf("Leg %d |	coxa: %f, femur: %f, tibia %f\n", this->id, target_angles.coxa, target_angles.femur, target_angles.tibia);

	// update members
	this->servo_angles_deg = target_angles_deg;
}

void Leg::updateStance(const float dt_s, const SharedData& shared_data) {
	// reset swing_phase for swing state
	this->swing_phase = 0.0f;

	// move leg backwards at body's current speed
	this->pos_state.foot_target_pos_lf.x += shared_data.current_speed_mm_s * dt_s * -1.0f;
}

void Leg::updateSwing(const float dt_s, const float step_length_mm, const float step_height_mm) {
	// swing done -> move to stance swing_phase
	if (this->swing_phase >= 1.0f) {
		setState(STANCE);
		return;
	}

	// x = velocity-based
	float velocity_x_mm_s = (PI * step_length_mm / (2.0f * Movement::k_swing_duration_s)) * sinf(PI * this->swing_phase);
	this->pos_state.foot_target_pos_lf.x += velocity_x_mm_s * dt_s;

	// z = position-based, add on top of whatever update_orientation() just set
	this->pos_state.foot_target_pos_lf.z = Offset::neutral_foot_pos_lf[this->id].z + step_height_mm * sinf(PI * this->swing_phase);

	// advance swing_phase
	this->swing_phase += dt_s / Movement::k_swing_duration_s;
	if (this->swing_phase > 1.0f) this->swing_phase = 1.0f;
}

void Leg::update(const float dt_s, const SharedData& shared_data, Theta3& out_target_angles_deg) {
	// 1. update target pos based on state
	if (this->leg_state == SWING) {
		// calc step length from where leg lifts
		float target_pos_x_lf = Offset::neutral_foot_pos_lf[this->id].x + (Movement::k_step_length_mm / 2.0f);
		float adjusted_step_length_mm = target_pos_x_lf - this->pos_state.foot_last_grounded_pos_lf.x;

		updateSwing(dt_s, adjusted_step_length_mm, Movement::k_step_height_mm);
	} else if (this->leg_state == STANCE) {
		updateStance(dt_s, shared_data);
	} else {
		this->pos_state.foot_target_pos_lf = Offset::neutral_foot_pos_lf[this->id]; // TODO: CHANGE LATER
	}

	// 2. while grounded, update pos to calculate proper step length
	if (isGrounded()) {
		this->pos_state.foot_last_grounded_pos_lf = this->pos_state.foot_target_pos_lf;
	}

	// 3. move servos based on target pos
	moveLeg();

	// update variables
	this->pos_state.foot_curr_pos_lf = this->pos_state.foot_target_pos_lf;
	out_target_angles_deg = this->servo_angles_deg;
}

float Leg::getSwingTimeRemainingS() const {
	if (this->leg_state != SWING) return 0.0f;
	return (1.0f - this->swing_phase) * Movement::k_swing_duration_s;
}

// Touchdown point of the current swing, leg frame. updateSwing() integrates x so the swing ends at
// foot_neutral.x + k_step_length_mm/2 (that is what `adjusted_step_length_mm` works out to), and z
// returns to the neutral z (sin(pi*1) = 0). y is not changed by the gait.
Vec3 Leg::getSwingEndTarget() const {
	Vec3 foot_end_target_lf = this->pos_state.foot_target_pos_lf;
	if (this->leg_state == SWING) {
		foot_end_target_lf.x = Offset::neutral_foot_pos_lf[this->id].x + Movement::k_step_length_mm / 2.0f;
		foot_end_target_lf.z = Offset::neutral_foot_pos_lf[this->id].z;
	}
	return foot_end_target_lf + this->pos_state.foot_offset_lf;
}