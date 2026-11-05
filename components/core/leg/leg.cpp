#include "leg.h"
#include "leg_controller.h"
#include "servo.h"
#include "types.h"
#include "ik.h"

static bool isRightLeg(const int id) {
	return id == 1 || id == 2;
}
static bool isFrontLeg(const int id) {
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

		.swing_start_pos_lf = neutral_foot_pos_lf,
		.swing_target_pos_lf = neutral_foot_pos_lf,

		.stance_start_pos_lf = neutral_foot_pos_lf,
		.stance_target_pos_lf = neutral_foot_pos_lf
	};

	this->swing_phase = 0.0f;
	this->stance_phase = 0.0f;
	this->leg_state = HOLD;

	moveLeg();
}

void Leg::moveLeg() {
	// flip y pos
	Vec3 adjusted_target_pos_lf = this->pos_state.foot_target_pos_lf;
	adjusted_target_pos_lf.y *= isRightLeg(id) ? -1.0f : 1.0f;

	Theta3 target_angles_deg;

	bool reachable = IK({
		.body_coxa_x_mm = BODY_COXA_X_MM,
		.body_coxa_z_mm = BODY_COXA_Z_MM,
		.femur_length_mm = FEMUR_LENGTH_MM,
		.tibia_length_mm = TIBIA_LENGTH_MM,
		.c_min = c_min,
		.c_max = c_max
	}, adjusted_target_pos_lf, target_angles_deg);

	if (reachable) {
		// flip angles for reverse mounted servos (leg side)
		if (!isRightLeg(id)) {
			target_angles_deg = invertAngles(target_angles_deg);
		}

		// clamp servos to calibrated limits
		target_angles_deg = clampToLimits(target_angles_deg, this->id);
		
		// printf("Leg %d |	coxa: %f, femur: %f, tibia %f\n", this->id, target_angles.coxa, target_angles.femur, target_angles.tibia);

		// update members
		this->servo_angles_deg = target_angles_deg;
	}
}

void Leg::startSwing(const Vec3& target_pos_lf) {
    this->pos_state.swing_start_pos_lf = this->pos_state.foot_target_pos_lf;
    this->pos_state.swing_target_pos_lf = target_pos_lf;
    this->swing_phase = 0.0f;
    setState(SWING);
}

void Leg::startStance(const Vec3& target_pos_lf) {
    this->pos_state.stance_start_pos_lf = this->pos_state.foot_target_pos_lf;
    this->pos_state.stance_target_pos_lf = target_pos_lf;
    this->stance_phase = 0.0f;
    setState(STANCE);
}

void Leg::updateStance(const float dt_s) {
	this->stance_phase += dt_s / (Movement::SWING_DURATION_S * 3.0f);

    if (this->stance_phase > 1.0f) {
        this->stance_phase = 1.0f;
    }

    const float t = this->stance_phase;

    const Vec3& start = this->pos_state.stance_start_pos_lf;
    const Vec3& target = this->pos_state.stance_target_pos_lf;

    // smooth horizontal interpolation
    const float alpha = t * t * (3.0f - 2.0f * t);

	this->pos_state.foot_target_pos_lf = start + (target - start) * alpha;

    if (t >= 1.0f) {
        // ensure the final target is exact
        this->pos_state.foot_target_pos_lf = target;
    }
}

void Leg::updateSwing(const float dt_s) {
    // advance and clamp swing progress
    this->swing_phase += dt_s / Movement::SWING_DURATION_S;

    if (this->swing_phase > 1.0f) {
        this->swing_phase = 1.0f;
    }

    const float t = this->swing_phase;

    const Vec3& start = this->pos_state.swing_start_pos_lf;
    const Vec3& target = this->pos_state.swing_target_pos_lf;

    // smooth horizontal interpolation
    const float alpha = t * t * (3.0f - 2.0f * t);

    this->pos_state.foot_target_pos_lf.x = start.x + alpha * (target.x - start.x);
    this->pos_state.foot_target_pos_lf.y = start.y + alpha * (target.y - start.y);

    // lift the foot above the start-to-target line
    const float clearance = Movement::STEP_HEIGHT_MM * sinf(PI * t);
    this->pos_state.foot_target_pos_lf.z = (start.z + t * (target.z - start.z)) + clearance;

    if (t >= 1.0f) {
        // ensure the final target is exact
        this->pos_state.foot_target_pos_lf = target;
		Vec3 foot_end_target_lf = this->pos_state.swing_start_pos_lf;
		foot_end_target_lf.x = Offset::neutral_foot_pos_lf[this->id].x - Movement::STEP_LENGTH_MM / 2.0f;
		startStance(foot_end_target_lf);
    }
}

void Leg::update(const float dt_s, Theta3& out_target_angles_deg, const ImuData& imu_data) {
	// update target pos based on state
	if (this->leg_state == SWING) {
		updateSwing(dt_s);
	} else if (this->leg_state == STANCE) {
		updateStance(dt_s);
		pidCorrection(imu_data);
	} else {
		this->pos_state.foot_target_pos_lf = Offset::neutral_foot_pos_lf[this->id]; // TODO: CHANGE LATER
	}

	// IK stuff
	moveLeg();

	// update variables
	this->pos_state.foot_curr_pos_lf = this->pos_state.foot_target_pos_lf;
	out_target_angles_deg = this->servo_angles_deg;
}

Vec3 Leg::getNominalLandingPos() const {
	Vec3 foot_end_target_lf = this->pos_state.swing_start_pos_lf;
	foot_end_target_lf.x = Offset::neutral_foot_pos_lf[this->id].x + Movement::STEP_LENGTH_MM / 2.0f;

	return foot_end_target_lf;
}

void Leg::pidCorrection(const ImuData& imu_data) {
	if (imu_data.valid) {
		this->pos_state.foot_target_pos_lf.rotate(imu_data.rot_mat);
	}
}