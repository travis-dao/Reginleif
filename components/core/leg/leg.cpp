#include "leg.h"
#include "leg_config.h"
#include "leg_controller.h"
#include "servo.h"
#include "types.h"
#include "ik.h"
#include <algorithm>
#include <cmath>

Theta3 clampToLimits(const Theta3& angles_deg, const int leg_idx) {
	const LegServoLimits& lim = ServoConfig::limits[leg_idx];

	return Theta3 {
		lim.coxa.clamp(angles_deg.coxa),
		lim.femur.clamp(angles_deg.femur),
		lim.tibia.clamp(angles_deg.tibia)
	};
}

bool outsideServoLimits(const Theta3& angles_deg, const int leg_idx) {
	const LegServoLimits& lim = ServoConfig::limits[leg_idx];

	if (angles_deg.coxa > lim.coxa.max_deg || angles_deg.coxa < lim.coxa.min_deg) return false;
	if (angles_deg.femur > lim.femur.max_deg || angles_deg.femur < lim.femur.min_deg) return false;
	if (angles_deg.tibia > lim.tibia.max_deg || angles_deg.tibia < lim.tibia.min_deg) return false;
	
	return true;
}

static bool isRightLeg(const int id) {
	return id == 1 || id == 2;
}
static bool isFrontLeg(const int id) {
	return id == 0 || id == 1;
}


Leg::Leg(const Config& cfg) {
	Vec3 neutral_foot_pos_lf = Offset::neutral_foot_pos_lf[cfg.id];

	id_ = cfg.id;
	pos_state_ = {
		.foot_curr_pos_lf = neutral_foot_pos_lf,
		.foot_target_pos_lf = neutral_foot_pos_lf,

		.swing_start_pos_lf = neutral_foot_pos_lf,
		.swing_target_pos_lf = neutral_foot_pos_lf,

		.stance_start_pos_lf = neutral_foot_pos_lf,
		.stance_target_pos_lf = neutral_foot_pos_lf
	};

	swing_phase_ = 0.0f;
	stance_phase_ = 0.0f;
	state_ = HOLD;

	moveLeg();
}

void Leg::moveLeg() {
	// flip y pos
	Vec3 adjusted_target_pos_lf = pos_state_.foot_target_pos_lf;
	adjusted_target_pos_lf.y *= isRightLeg(id_) ? -1.0f : 1.0f;

	Theta3 target_angles_deg;

	bool reachable = IK({
		.body_coxa_x_mm = BODY_COXA_X_MM,
		.femur_length_mm = FEMUR_LENGTH_MM,
		.tibia_length_mm = TIBIA_LENGTH_MM,
		.c_min = c_min,
		.c_max = c_max
	}, adjusted_target_pos_lf, target_angles_deg);

	if (reachable) {
		// flip angles for reverse mounted servos (leg side)
		if (!isRightLeg(id_)) {
			target_angles_deg = invertAngles(target_angles_deg);
		}

		// clamp servos to calibrated limits
		target_angles_deg = clampToLimits(target_angles_deg, id_);
		
		// printf("Leg %d |	coxa: %f, femur: %f, tibia %f\n", id_, target_angles.coxa, target_angles.femur, target_angles.tibia);

		// update members
		servo_angles_deg_ = target_angles_deg;
	}
}

void Leg::startSwing(const Vec3& target_pos_lf) {
    pos_state_.swing_start_pos_lf = pos_state_.foot_target_pos_lf;
    pos_state_.swing_target_pos_lf = target_pos_lf;
    swing_phase_ = 0.0f;
    setState(SWING);
}

void Leg::startStance(const Vec3& target_pos_lf) {
    pos_state_.stance_start_pos_lf = pos_state_.foot_target_pos_lf;
    pos_state_.stance_target_pos_lf = target_pos_lf;
    stance_phase_ = 0.0f;
    setState(STANCE);
}

void Leg::updateStance(const float dt_s) {
	Vec3 curr_foot_pos_lf = pos_state_.stance_target_pos_lf;
	float rear_foot_x_lf = pos_state_.stance_target_pos_lf.x;

	curr_foot_pos_lf.x = std::max(curr_foot_pos_lf.x - stanceSpeed() * dt_s, rear_foot_x_lf);

	float start_foot_x_lf = pos_state_.stance_start_pos_lf.x;
	float span = start_foot_x_lf - rear_foot_x_lf;
	stance_phase_ = (span < EPSILON) ? (1.0f) : fmin((start_foot_x_lf - curr_foot_pos_lf.x) / span, 1.0f);
}

void Leg::updateSwing(const float dt_s) {
    // advance and clamp swing progress
    swing_phase_ += dt_s / Movement::SWING_DURATION_S;

    if (swing_phase_ > 1.0f) {
        swing_phase_ = 1.0f;
    }

    const float t = swing_phase_;

    const Vec3& start = pos_state_.swing_start_pos_lf;
    const Vec3& target = pos_state_.swing_target_pos_lf;

    // smooth step
    const float alpha = t * t * (3.0f - 2.0f * t);

    pos_state_.foot_target_pos_lf.x = start.x + alpha * (target.x - start.x);
    pos_state_.foot_target_pos_lf.y = start.y + alpha * (target.y - start.y);

    // lift the foot above the start-to-target line
    const float clearance = Movement::STEP_HEIGHT_MM * pow(sinf(PI * t), 2);
    pos_state_.foot_target_pos_lf.z = (start.z + t * (target.z - start.z)) + clearance;

    if (t >= 1.0f) {
		pos_state_.foot_target_pos_lf = target;
		startStance(rearFootPos());
    }
}

void Leg::update(const float dt_s, Theta3& out_target_angles_deg, const ImuData& imu_data) {
	// update target pos based on state
	if (state_ == SWING) {
		updateSwing(dt_s);
	} else if (state_ == STANCE) {
		updateStance(dt_s);
		applyPidCorrection(imu_data);
	} else {
		pos_state_.foot_target_pos_lf = Offset::neutral_foot_pos_lf[id_]; // TODO: CHANGE LATER
	}

	// IK stuff
	moveLeg();

	Vec3 mpc_offset_cmd = state_ == LegState::SWING ? Vec3 {0.0f, 0.0f, 0.0f} : pos_state_.mpc_offset_cmd;
	float alpha = 1.0 - std::exp(-dt_s / MPC_OFFSET_TAU_S);
	pos_state_.mpc_offset_lf += (mpc_offset_cmd - pos_state_.mpc_offset_lf) * alpha;

	// update variables
	pos_state_.foot_curr_pos_lf = pos_state_.foot_target_pos_lf + pos_state_.mpc_offset_lf;
	out_target_angles_deg = servo_angles_deg_;
}

Vec3 Leg::nominalLandingPos() const {
	Vec3 foot_end_target_lf = Offset::neutral_foot_pos_lf[id_];
	foot_end_target_lf.x += Movement::STEP_LENGTH_MM / 2.0f;
	return foot_end_target_lf;
}

void Leg::applyPidCorrection(const ImuData& imu_data) {
	if (imu_data.valid) {
		pos_state_.foot_target_pos_lf.rotate(imu_data.rot_mat);
	}
}

float Leg::stanceSpeed() {
	return Movement::STEP_LENGTH_MM / (Movement::SWING_DURATION_S * 3.0f + Movement::GAP_DURATION_S * 4.0f);
}

Vec3 Leg::rearFootPos() const {
	Vec3 rear_foot_pos_lf = Offset::neutral_foot_pos_lf[id_];
	rear_foot_pos_lf.x -= Movement::STEP_LENGTH_MM / 2.0f;

	return rear_foot_pos_lf;
}