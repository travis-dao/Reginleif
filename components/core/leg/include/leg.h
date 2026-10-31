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

    Vec3 swing_start_pos_lf;
    Vec3 swing_target_pos_lf;

    Vec3 stance_start_pos_lf;
    Vec3 stance_target_pos_lf;
};

class Leg {
	public:
		struct Config {
			int id;
			Pca9685& pca;
		};

		Leg(const Config& cfg);

		void update(const float dt_s, Theta3& out_target_angles_deg);
		void startSwing(const Vec3& target_pos_lf);
		void startStance(const Vec3& target_pos_lf);

		static bool IK(Vec3 target_foot_pos_lf, Theta3& out_angles_deg);
		static bool outsideServoLimits(const Theta3& angles_deg, const int id);

		void setState(const LegState state) { this->leg_state = state; }

		bool isGrounded() const { return this->leg_state != SWING; }
		
        float getSwingPhase() const { return this->swing_phase; }
		Vec3 getNominalLandingPos() const;
		Vec3 getCurrentLegPos() const { return this->pos_state.foot_curr_pos_lf; }

	private:
		Theta3 servo_angles_deg;
		float swing_phase;
		float stance_phase;

		int id;
		LegState leg_state;
		FootPositionState pos_state;

		Pca9685& pca;

		Theta3 invertAngles(const Theta3& angles_deg);
		Theta3 clampToLimits(const Theta3& angles_deg);
		void moveLeg();

		void updateSwing(const float dt_s);
		void updateStance(const float dt_s);
};

#endif