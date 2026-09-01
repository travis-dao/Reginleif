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
	constexpr Vec3 neutral_offset = { 90.0f, 100.0f, -50.0f};

	const Vec3 neutral_vector[4] = { 
		{ 1, 1, 1},  // Front Left
		{ 1, -1, 1},   // Front Right
		{ -1, -1, 1},  // Back Right
		{ -1, 1, 1}, // Back Left
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

enum LegState {
 	HOLD, SWING, STANCE
};

struct Info {
	int id;
	bool is_front_leg;
	bool is_right_leg;

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

		Theta3 ik(float x, float y, float z);
		static Theta3 get_inverted_angles(Theta3 angles);

		void move_leg();

		void update_swing(const float step_length, const float step_height);
		void update_stance();
		void update_orientation();

	public:
		Leg(int id);
		void update();

		void update_state(const LegState state) {
			this->state = state;
		}
		
		bool is_grounded() {
			return state != SWING;
		}
};

#endif