#ifndef LEG_h
#define LEG_h

#include "helpers.h"
#include "types.h"

enum LegState {
 	HOLD, SWING, STANCE
};

struct Info {
	int id;
	bool is_front_leg;
	bool is_right_leg;

	Info(int id);
};

struct PositionState {
	Vec3 base_neutral_pos;
	Vec3 true_neutral_pos;
	Vec3 curr_pos;
	Vec3 target_pos;
	Vec3 last_grounded_pos;
};

class Leg {
	private:
		float orientation_offset;
		Theta3 angles;
		float phase;

		Info info;
		LegState leg_state;
		PositionState pos_state;

		Theta3 ik(Vec3 target_foot_pos);
		Theta3 get_inverted_angles(Theta3 angles);
		Theta3 clamp_to_limits(Theta3 angles);
		void move_leg();

		void update_swing(const float step_length, const float step_height);
		void update_stance();
		void update_orientation();

		void apply_pid_stabilization();
		void balance(const Vec3& angles);

	public:
		Leg(int id);

		void update();
		void update_state(const LegState state) { this->leg_state = state; }

		bool is_grounded() { return leg_state != SWING; }
};

#endif