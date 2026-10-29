#ifndef LEG_h
#define LEG_h

#include "pca9685.h"
#include "types.h"
#include "stabilizer.h"

enum LegState {
 	HOLD, SWING, STANCE
};

struct FootPositionState {
	Vec3 foot_curr_pos_lf;
	Vec3 foot_target_pos_lf;
	Vec3 foot_last_grounded_pos_lf;
	Vec3 foot_offset_lf;
};

class Leg {
	public:
		struct Config {
			int id;
			Pca9685& pca;
		};

		Leg(const Config& cfg);

		void update(float dt_s, const SharedData& shared_data, Theta3& out_target_angles_deg);

		void setState(const LegState state) { this->leg_state = state; }
		void setFootOffset(const Vec3 offset) { this->pos_state.foot_offset_lf = offset; }

		bool isGrounded() const { return this->leg_state != SWING; }
		
        float getSwingPhase() const { return this->swing_phase; }
		float getSwingTimeRemainingS() const;
		Vec3 getSwingEndTarget() const;
		// leg frame
		Vec3 getFootTarget() const { return this->pos_state.foot_target_pos_lf; }

	private:
		Theta3 servo_angles_deg;
		float swing_phase;

		int id;
		LegState leg_state;
		FootPositionState pos_state;

		Pca9685& pca;

		Theta3 IK(Vec3 target_foot_pos_lf);
		Theta3 invertAngles(const Theta3& angles_deg);
		Theta3 clampToLimits(const Theta3& angles_deg);
		void moveLeg();

		void updateSwing(const float dt_s, const float step_length_mm, const float step_height_mm);
		void updateStance(const float dt_s, const SharedData& shared_data);
};

#endif