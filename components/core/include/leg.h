#ifndef LEG_h
#define LEG_h

#include "pca9685.h"
#include "types.h"
#include "pid_stabilize.h"

enum LegState {
 	HOLD, SWING, STANCE
};

struct Info {
	int id;
	bool is_front_leg;
	bool is_right_leg;

	Info(int id) {
		this->id = id;
		this->is_front_leg = id == 0 || id == 1;
		this->is_right_leg = id == 1 || id == 2;
	}
};

struct PositionState {
	Vec3 base_neutral_pos;
	Vec3 true_neutral_pos;
	Vec3 curr_pos;
	Vec3 target_pos;
	Vec3 last_grounded_pos;
};

class Base;

class Leg {
	public:
		struct Config {
			int id;
			Base& base_;
			pca9685::Pca9685& pca;
			Stabilizer& stabilizer_;
		};

		Leg(const Config& cfg);

		void update();
		void update_state(const LegState state) { this->leg_state = state; }

		bool is_grounded() { return leg_state != SWING; }

	private:
		float orientation_offset;
		Theta3 angles;
		float phase;

		LegState leg_state;
		PositionState pos_state;
		Info info;

		Base& base_;
		pca9685::Pca9685& pca;
		Stabilizer& stabilizer_;

		Theta3 ik(Vec3 target_foot_pos);
		Theta3 invert_angles(const Theta3& angles);
		Theta3 clamp_to_limits(const Theta3& angles) const;
		void move_leg();

		void update_swing(const float step_length, const float step_height);
		void update_stance();
		void update_orientation();

		void apply_pid_stabilization();
		void balance(const Vec3& angles);
};

#endif