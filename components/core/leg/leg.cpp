#include "leg.h"
#include "leg_controller.h"
#include "servo.h"
#include "types.h"

constexpr float k_femur_length_mm = 122.64f;
constexpr float k_tibia_length_mm = 34.66f + 147.03f;
constexpr float k_body_coxa_x_mm = 28.58f;
constexpr float k_body_coxa_z_mm = 0.0f;

const float c_max = k_femur_length_mm + k_tibia_length_mm;
const float c_min = fabsf(k_femur_length_mm - k_tibia_length_mm);

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

Theta3 Leg::invertAngles(const Theta3& angles_deg) {
	return Theta3 {
		180.0f - angles_deg.coxa,
		180.0f - angles_deg.femur,
		180.0f - angles_deg.tibia
	};
}

Theta3 Leg::clampToLimits(const Theta3& angles_deg) {
	const LegServoLimits& lim = ServoConfig::limits[this->id];

	return Theta3 {
		lim.coxa.clamp(angles_deg.coxa),
		lim.femur.clamp(angles_deg.femur),
		lim.tibia.clamp(angles_deg.tibia)
	};
}

bool Leg::outsideServoLimits(const Theta3& angles_deg, const int id) {
	const LegServoLimits& lim = ServoConfig::limits[id];

	if (angles_deg.coxa > lim.coxa.max_deg || angles_deg.coxa < lim.coxa.min_deg) return false;
	if (angles_deg.femur > lim.femur.max_deg || angles_deg.femur < lim.femur.min_deg) return false;
	if (angles_deg.tibia > lim.tibia.max_deg || angles_deg.tibia < lim.tibia.min_deg) return false;
	
	return true;
}

bool Leg::IK(Vec3 target_foot_pos_lf, Theta3& out_angles_deg) {
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

	// reachability check
	if (c > c_max + k_epsilon || c < c_min - k_epsilon || c < k_epsilon) {
		return false;  // unreachable, leave out_angles_deg untouched
	}

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

	out_angles_deg = target_angles_deg;
	return true;
}

void Leg::moveLeg() {
	// flip y pos
	Vec3 adjusted_target_pos_lf = this->pos_state.foot_target_pos_lf;
	adjusted_target_pos_lf.y *= isRightLeg(id) ? -1.0f : 1.0f;

	Theta3 target_angles_deg;
	bool reachable = IK(adjusted_target_pos_lf, target_angles_deg);

	if (reachable) {
		// flip angles for reverse mounted servos (leg side)
		if (!isRightLeg(id)) {
			target_angles_deg = invertAngles(target_angles_deg);
		}

		// clamp servos to calibrated limits
		target_angles_deg = clampToLimits(target_angles_deg);
		
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
	this->stance_phase += dt_s / (Movement::k_swing_duration_s * 3.0f);

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
    this->swing_phase += dt_s / Movement::k_swing_duration_s;

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
    const float clearance = Movement::k_step_height_mm * sinf(k_pi * t);
    this->pos_state.foot_target_pos_lf.z = (start.z + t * (target.z - start.z)) + clearance;

    if (t >= 1.0f) {
        // ensure the final target is exact
        this->pos_state.foot_target_pos_lf = target;
		Vec3 foot_end_target_lf = this->pos_state.swing_start_pos_lf;
		foot_end_target_lf.x = Offset::neutral_foot_pos_lf[this->id].x - Movement::k_step_length_mm / 2.0f;
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
	foot_end_target_lf.x = Offset::neutral_foot_pos_lf[this->id].x + Movement::k_step_length_mm / 2.0f;

	return foot_end_target_lf;
}

void Leg::pidCorrection(const ImuData& imu_data) {
	if (imu_data.valid) {
		this->pos_state.foot_target_pos_lf.rotate(imu_data.rot_mat);
	}
}