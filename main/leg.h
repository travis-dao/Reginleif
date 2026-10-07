#ifndef LEG_h
#define LEG_h

#include "helpers.h"

namespace LegConfig {
	constexpr float femur_length = 89.806f;
	constexpr float tibia_length = 141.701f;
	constexpr float body_to_coxa_x_offset = 45.125f;
	constexpr float body_to_coxa_z_offset = -10.0f;
}

struct Theta3 {
  	float coxa, femur, tibia;
};

namespace NeutralConfig {
	constexpr Vec3 neutral_offset = { 90.0f, 100.0f, -50.0f };

	const Vec3 neutral_vector[4] = { 
		{ 1, 1, 1 },  // Front Left
		{ 1, -1, 1 },   // Front Right
		{ -1, -1, 1 },  // Back Right
		{ -1, 1, 1 }, // Back Left
	};
}

// how much to move other legs up/down during stabilization
constexpr float WALK_OPPOSITE_LEG_Z_OFFSET[4] = { 5.0f, 5.0f, 10.0f, 10.0f };
constexpr float WALK_ADJACENT_LEG_Z_OFFSET[4] = { -8.0f, -8.0f, -6.0f, -6.0f };

namespace MovementConfig {
	// constexpr float MAX_STEP_LENGTH_MM = 80.0f;
	constexpr float MAX_STEP_LENGTH_MM = 0.0f;
	constexpr float STEP_HEIGHT_MM = 100.0f;    // mm, max lift height
	constexpr float SWING_DURATION_S = 0.250f;   // s, swing duration
}

namespace PIDConfig {
	// PID
	constexpr float K_p = 1.5;
	constexpr float K_i = 0.0;
	constexpr float K_d = 0.0;

	constexpr float INTEGRAL_LIMIT_DEG = 30.0;  // max contribution of integral_sum, in degrees
	constexpr float COMPENSATION_LIMIT_DEG = 30.0; // max total correction angle, in degrees
}

enum LegState {
 	HOLD, SWING, STANCE
};

struct Info {
	int id;
	bool is_front_leg;
	bool is_right_leg;


	/**
	* @brief Constructs an Info struct describing a leg's identity and position on the body.
	*
	* @param id Numeric identifier of the leg (0–3).
	*/
	Info(int id);
};

class Leg {
	private:
		Vec3 curr_pos;
		Vec3 target_pos;
		Vec3 last_grounded_pos;
		Vec3 base_neutral_pos;
		Vec3 true_neutral_pos;
		
		float orientation_offset;
		Theta3 angles;
		float phase;

		Info info;
		LegState state;

		/**
		* @brief Computes inverse kinematics joint angles for a target foot position.
		*
		* @param target_foot_pos Vec3 The target food position (tip of the leg) with respect to the relative position of the leg
		* @return Theta3 The computed coxa, femur, and tibia joint angles (degrees).
		*/
		Theta3 ik(Vec3 target_foot_pos);

		/**
		* @brief Inverts joint angles for legs with reverse-mounted servos (left side).
		*
		* @param out_angles The originally computed coxa/femur/tibia angles.
		* @return Theta3 The inverted angles suitable for reverse-mounted servos.
		*/
		static Theta3 get_inverted_angles(Theta3 angles);

		/**
		* @brief Converts the leg's target position into servo angles and drives the servos.
		*/
		void move_leg();

		/**
		* @brief Updates the leg's target position while in the SWING state.
		*
		* @param step_length Horizontal distance the foot should travel during the swing.
		* @param step_height Maximum vertical lift height during the swing arc.
		*/
		void update_swing(const float step_length, const float step_height);

		/**
		* @brief Updates the leg's target position while in the STANCE state.
		*/
		void update_stance();

		/**
		* @brief Adjusts the leg's target height to compensate for body orientation.
		*
		* @note PID controller migration/tuning is still pending.
		*/
		void update_orientation();

		void apply_pid_stabilization();
		void balance(const Vec3& angles);

	public:
		/**
		* @brief Constructs a Leg object and initializes its neutral position and state.
		*
		* @param id Numeric identifier of the leg (0–3), passed through to Info.
		*/
		Leg(int id);

		/**
		* @brief Runs one full per-leg update cycle.
		*/
		void update();

		void update_state(const LegState state) {
			this->state = state;
		}

		/**
		* @brief Checks if leg is ground, or when leg is not in swing state.
		*/
		bool is_grounded() {
			return state != SWING;
		}
};

#endif