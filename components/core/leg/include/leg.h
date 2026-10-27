#ifndef LEG_h
#define LEG_h

#include "pca9685.h"
#include "types.h"
#include "imu_wrapper.h"

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
	Vec3 curr_pos;
	Vec3 target_pos;
	Vec3 last_grounded_pos;
};

class Leg {
	public:
		struct Config {
			int id;
			Pca9685& pca;
		};

		Leg(const Config& cfg);

		void update(float dt_s, const SharedData& shared_data, Theta3& out_target_angles);
		void setState(const LegState state) { this->leg_state = state; }

		bool isGrounded() { return leg_state != SWING; }
        float getSwingPhase() { return phase; }

	private:
		float orientation_offset;
		Theta3 angles;
		float phase;

		LegState leg_state;
		PositionState pos_state;
		Info info;

		Pca9685& pca;

		Theta3 IK(Vec3 target_foot_pos);
		Theta3 invertAngles(const Theta3& angles);
		Theta3 clampToLimits(const Theta3& angles);
		void moveLeg();

		void updateSwing(const float dt_s, const float step_length, const float step_height);
		void updateStance(const float dt_s, const SharedData& shared_data);
};

#endif