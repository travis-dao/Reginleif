#include "ik.h"
#include <algorithm>

bool IK(const IKConfig& cfg, Vec3 target_foot_pos_lf, Theta3& out_angles_deg) {
	Theta3 target_angles_deg;

	// dis to target pos on x-y plane
	float d = sqrt(target_foot_pos_lf.x * target_foot_pos_lf.x + target_foot_pos_lf.y * target_foot_pos_lf.y);

	// adjust dis for offset to where coxa servo connects femur servo
	float r = d - cfg.body_coxa_x_mm;

	// dis to target pos on x-z plane, basically dis from femur servo to tip of tibia
	float femur_to_tip = sqrt(target_foot_pos_lf.z * target_foot_pos_lf.z + r * r);

	float c_squared = femur_to_tip * femur_to_tip;
	float a_squared = cfg.femur_length_mm * cfg.femur_length_mm;
	float b_squared = cfg.tibia_length_mm * cfg.tibia_length_mm;

	// reachability check
	if (femur_to_tip > cfg.c_max + EPSILON || femur_to_tip < cfg.c_min - EPSILON || femur_to_tip < EPSILON) {
		return false;  // unreachable, leave out_angles_deg untouched
	}

	// calculate femur servo
	float cos2 = std::clamp(
		(a_squared + c_squared - b_squared) / (2 * cfg.femur_length_mm * femur_to_tip), 
		-1.0f, 
		1.0f
	);

	// calculate tibia servo
	float cos3 = std::clamp(
		(a_squared + b_squared - c_squared) / (2 * cfg.femur_length_mm * cfg.femur_length_mm), 
		-1.0f, 
		1.0f
	);

	// update output
	target_angles_deg.coxa = atan2f(target_foot_pos_lf.y, target_foot_pos_lf.x) * RAD_TO_DEG; // coxa
	target_angles_deg.femur = atan2f(r, -target_foot_pos_lf.z) * RAD_TO_DEG + acosf(cos2) * RAD_TO_DEG; // femur
	target_angles_deg.tibia = acosf(cos3) * RAD_TO_DEG; // tibia

	out_angles_deg = target_angles_deg;
	return true;
}

Theta3 invertAngles(const Theta3& angles_deg) {
	return Theta3 {
		180.0f - angles_deg.coxa,
		180.0f - angles_deg.femur,
		180.0f - angles_deg.tibia
	};
}