#include "leg.h"
#include "leg_config.h"
#include "leg_controller.h"
#include "servo.h"
#include "types.h"
#include "ik.h"
#include <cmath>

// --------------- helpers ---------------
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

static bool isRightLeg(const int id) { return id == 1 || id == 2; }
static bool isFrontLeg(const int id) { return id == 0 || id == 1; }


// --------------- init ---------------
Leg::Leg(const Config& cfg) {
	Vec3 neutral_foot_pos_lf = Offset::neutral_foot_pos_lf[cfg.id];

	id_ = cfg.id;
	foot_state_ = {
		.curr = neutral_foot_pos_lf,
		.target = neutral_foot_pos_lf,

		.swing_start = neutral_foot_pos_lf,
		.swing_target = neutral_foot_pos_lf,

		.stance_start = neutral_foot_pos_lf,
		.stance_target = neutral_foot_pos_lf,

		.offset = Vec3{0.0f, 0.0f, 0.0f},
		.offset_cmd = Vec3{0.0f, 0.0f, 0.0f},
	};

	swing_phase_ = 0.0f;
	stance_phase_ = 0.0f;
	state_ = NEUTRAL;

	moveLeg();
}


// --------------- movement ---------------
void Leg::moveLeg() {
	// flip y pos
	Vec3 adjusted_target_pos_lf = foot_state_.target;
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
		angles = target_angles_deg;
	}
}


// --------------- commands ---------------
void Leg::startSwing(const Vec3& target_pos_lf) {
    foot_state_.swing_start = foot_state_.target;
    foot_state_.swing_target = target_pos_lf;
    swing_phase_ = 0.0f;
    setState(SWING);
}

void Leg::startStance(const Vec3& target_pos_lf) {
    foot_state_.stance_start = foot_state_.target;
    foot_state_.stance_target = target_pos_lf;
    stance_phase_ = 0.0f;
    setState(STANCE);
}

void Leg::hold() {
	if (state_ == SWING) return;
	setState(PAUSE);
}

void Leg::resumeStance() {
	startStance(rearFootPos());
}


// --------------- update ---------------
void Leg::updateStance(const float dt_s) {
	Vec3 start = foot_state_.stance_start;
	Vec3 foot = foot_state_.stance_target;
	Vec3 rear = rearFootPos();

	// continue sweeping while rear limit is ahead of foot
	if ((rear.x - foot.x) * vel_state_.stance_vel.x + (rear.x - foot.x) * vel_state_.stance_vel.y > 0.0f) {
		foot.x += vel_state_.stance_vel.x * dt_s;
		foot.y += vel_state_.stance_vel.y * dt_s;
	}

	float span = rear.dis(start);
	float done = foot.dis(start);
	stance_phase_ = (span < EPSILON) ? (1.0f) : fminf(done / span, 1.0f);
}

void Leg::updateSwing(const float dt_s) {
    // advance and clamp swing progress
    swing_phase_ += dt_s / MovementConfig::SWING_DURATION_S;

    if (swing_phase_ > 1.0f) {
        swing_phase_ = 1.0f;
    }

    const float t = swing_phase_;

    const Vec3& start = foot_state_.swing_start;
    const Vec3& target = foot_state_.swing_target;

    // smooth step
    const float alpha = t * t * (3.0f - 2.0f * t);

    foot_state_.target.x = start.x + alpha * (target.x - start.x);
    foot_state_.target.y = start.y + alpha * (target.y - start.y);

    // lift the foot above the start-to-target line
    const float clearance = MovementConfig::STEP_HEIGHT_MM * pow(sinf(PI * t), 2);
    foot_state_.target.z = (start.z + t * (target.z - start.z)) + clearance;

    if (t >= 1.0f) {
		foot_state_.target = target;
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
	} else if (state_ == PAUSE) {
		// do nothing
	} else { // HOLD
		foot_state_.target = Offset::neutral_foot_pos_lf[id_]; // TODO: CHANGE LATER
	}

	// update mpc offset calc
	Vec3 offset_cmd = state_ == SWING ? Vec3 {0.0f, 0.0f, 0.0f} : foot_state_.offset_cmd;
	float alpha = 1.0 - exp(-dt_s / MPC_OFFSET_TAU_S);
	foot_state_.offset += (offset_cmd - foot_state_.offset) * alpha;

	moveLeg();

	// update variables
	foot_state_.curr = foot_state_.target + foot_state_.offset;
	out_target_angles_deg = angles;
}


// --------------- setters ---------------
void Leg::setMpcOffset(const Vec3& offset_lf) { foot_state_.offset_cmd = offset_lf; }
void Leg::setStance(const Vec3& vel, const Vec3& step) { vel_state_.stance_vel = vel; vel_state_.step_vec = step; }
void Leg::setState(const LegState state) { state_ = state; }


// --------------- getters ---------------
bool Leg::grounded() const { return state_ != SWING; }
float Leg::swingTimeRemainingS() const {
	if (state_ == LegState::SWING) return 0.0f;
	else return (1.0f - swing_phase_) * MovementConfig::SWING_DURATION_S;
}
Vec3 Leg::nominalLandingPos() const {
	Vec3 target = Offset::neutral_foot_pos_lf[id_];
	target.x += vel_state_.step_vec.x / 2.0f;
	target.y += vel_state_.step_vec.y / 2.0f;
	return target;
}
Vec3 Leg::rearFootPos() const {
	Vec3 rear = Offset::neutral_foot_pos_lf[id_];
	rear.x -= vel_state_.step_vec.x / 2.0f;
	rear.y -= vel_state_.step_vec.y / 2.0f;

	return rear;
}
Vec3 Leg::currFootPos() const { return foot_state_.curr; }

// --------------- pid ---------------
void Leg::applyPidCorrection(const ImuData& imu_data) {
	if (imu_data.valid) foot_state_.target.rotate(imu_data.rot_mat);
}