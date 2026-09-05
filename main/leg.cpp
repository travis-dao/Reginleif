#include <math.h>
#include <algorithm>

#include <mpu6050.h>

#include "leg.h"
#include "helpers.h"
#include "base.h"

/**
 * @brief Constructs an Info struct describing a leg's identity and position on the body.
 *
 * Stores the leg's numeric ID and derives whether it is a front leg
 * (IDs 0 or 1) and whether it is a right-side leg (IDs 1 or 2).
 *
 * @param id Numeric identifier of the leg (0–3).
 */
Info::Info(int id) {
	this->id = id;
	this->is_front_leg = id == 0 || id == 1;
	this->is_right_leg = id == 1 || id == 2;
}

/**
 * @brief Constructs a Leg object and initializes its neutral position and state.
 *
 * Computes the leg's base neutral position (scaled by neutral_offset)
 * and true neutral position (scaled by neutral_offset plus body_offset)
 * from the per-leg neutral vector. Initializes current, target, and
 * last-grounded positions to the base neutral position, resets the
 * gait phase to zero, and sets the initial state to HOLD. Moves leg to
 * starting positions (neutrals).
 *
 * @param id Numeric identifier of the leg (0–3), passed through to Info.
 */
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

/**
 * @brief Inverts joint angles for legs with reverse-mounted servos (left side).
 *
 * Left-side servos are physically mounted in reverse, so their computed
 * angles must be mirrored (180° minus the angle) to produce correct
 * physical motion.
 *
 * @param out_angles The originally computed coxa/femur/tibia angles.
 * @return Theta3 The inverted angles suitable for reverse-mounted servos.
 */
Theta3 Leg::get_inverted_angles(Theta3 out_angles) {
	out_angles.coxa = 180.0f - out_angles.coxa;
	out_angles.femur = 180.0f - out_angles.femur;
	// out_angles.tibia = 180.0f - out_angles.tibia;

	return out_angles;
}

/**
 * @brief Computes inverse kinematics joint angles for a target foot position.
 *
 * Given a target (x, y, z) position relative to the leg, calculates the
 * coxa angle from the horizontal projection, then uses the law of
 * cosines on the femur/tibia triangle (accounting for the coxa-to-body
 * offsets) to compute the femur and tibia angles needed to reach that
 * position.
 *
 * @param x Target x-coordinate of the foot.
 * @param y Target y-coordinate of the foot.
 * @param z Target z-coordinate of the foot.
 * @return Theta3 The computed coxa, femur, and tibia joint angles (degrees).
 */
Theta3 Leg::ik(float x, float y, float z) {
	Theta3 new_angles;

	// dis to target pos on x-y plane
	float d = sqrt(x * x + y * y);

	// adjust dis for offset to where coxa servo connects femur servo
	float r = d - LegConfig::body_to_coxa_x_offset;

	// dis to target pos on x-z plane, basically dis from femur servo to tip of tibia
	z += LegConfig::body_to_coxa_z_offset;
	float c = sqrt(z * z + r * r);

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
	new_angles.coxa = atan2(y, x) * 180.0f / M_PI; // coxa
	new_angles.femur = atan2(r, -z) * 180.0f / M_PI + acos(cos2) * 180.0f / M_PI; // femur
	new_angles.tibia = 180.0 - acos(cos3) * 180.0f / M_PI; // tibia

	return new_angles;
}

/**
 * @brief Converts the leg's target position into servo angles and drives the servos.
 *
 * Adjusts the target position for right-side leg mirroring, runs inverse
 * kinematics to get joint angles, flips those angles if needed for
 * reverse-mounted (left-side) servos, logs the result, and sends PWM
 * commands to the corresponding PCA9685 channels. Updates the stored
 * angles and current position to reflect the newly commanded pose.
 */
void Leg::move_leg() {
	Vec3 adjusted = this->target_pos;

	adjusted.y *= this->info.is_right_leg ? -1 : 1;

	Theta3 target_angles = ik(adjusted.x, adjusted.y, adjusted.z);

	// flip angles cuz servos on left leg flipped
	// target_angles = this->info.is_right_leg ? target_angles : get_inverted_angles(target_angles);

	if (this->info.is_right_leg) {
		target_angles.tibia = 180.0f - target_angles.tibia;
	} else {
		target_angles.coxa = 180.0f - target_angles.coxa;
		target_angles.femur = 180.0f - target_angles.femur;

	}
	
	printf("Leg %d |	coxa: %f, femur: %f, theta3 %f\n", this->info.id, target_angles.coxa, target_angles.femur, target_angles.tibia);

	// move servos
	pca9685_set_pwm_value(&Base::pca, this->info.id * 3,     angle_to_pulse(target_angles.coxa));
	pca9685_set_pwm_value(&Base::pca, this->info.id * 3 + 1, angle_to_pulse(target_angles.femur));
	pca9685_set_pwm_value(&Base::pca, this->info.id * 3 + 2, angle_to_pulse(target_angles.tibia));

	// update members
	this->angles = target_angles;
	this->curr_pos = this->target_pos;
}

/**
 * @brief Adjusts the leg's target height to compensate for body orientation.
 *
 * Computes a z-offset based on the body's target roll/pitch (via tangent
 * of the target orientation angles) and this leg's true neutral x/y
 * position, so that the leg raises or lowers to help level the body.
 * Applies this offset on top of the leg's base neutral z position.
 *
 * @note PID controller migration/tuning is still pending.
 */
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

/**
 * @brief Updates the leg's target position while in the STANCE state.
 *
 * Resets the gait phase to zero and shifts the target x-position
 * backward relative to the body, based on the body's current speed
 * and elapsed time step, simulating the leg pushing the body forward
 * while planted on the ground.
 */
void Leg::update_stance() {
	// reset phase for swing state
	this->phase = 0.0f;

	// move leg backwards at body's current speed
	target_pos.x += base.get_speed() * base.get_dt_s() * -1;
}

/**
 * @brief Updates the leg's target position while in the SWING state.
 *
 * Advances the foot along a sinusoidal velocity profile in x (forward
 * step motion) and a sinusoidal height profile in z (lift arc), based
 * on the given step length and step height. Advances the gait phase
 * each call; once the phase reaches 1.0, transitions the leg into the
 * STANCE state.
 *
 * @param step_length Horizontal distance the foot should travel during the swing.
 * @param step_height Maximum vertical lift height during the swing arc.
 */
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

/**
 * @brief Runs one full per-leg update cycle.
 *
 * Updates the orientation-based height offset, then updates the target
 * position according to the leg's current state (SWING, STANCE, or
 * otherwise resets to base neutral). If the leg is currently grounded,
 * records its position as the new last-grounded reference (used for
 * step-length calculations). Finally, converts the resulting target
 * position into servo commands via move_leg().
 */
void Leg::update() {
	// 1. update orientation offset and target_pos.z based on body's orientation
	update_orientation();

	// 2. update target pos based on state
	if (this->state == SWING) {
		// calc step length from where leg lifts
		float target_pos_x = this->base_neutral_pos.x + (MovementConfig::MAX_STEP_LENGTH_MM / 2);
		float adjusted_step_length = target_pos_x - last_grounded_pos.x;

		update_swing(adjusted_step_length, MovementConfig::STEP_HEIGHT_MM);
	} else if (this->state == STANCE) {
		update_stance();
	} else {
		this->target_pos = this->base_neutral_pos; // TODO: CHANGE LATER
	}

	// 3. while grounded, update pos to calculate proper step length
	if (is_grounded()) {
		this->last_grounded_pos = this->target_pos;
	}

	// 4. move servos based target pos
	move_leg();
}