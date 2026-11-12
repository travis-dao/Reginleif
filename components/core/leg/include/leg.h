#ifndef LEG_h
#define LEG_h

#include "leg_config.h"
#include "rtos_utils.h"
#include "types.h"

enum LegState {
 	NEUTRAL, SWING, STANCE, PAUSE
};

struct FootState {
	Vec3 curr;
	Vec3 target;

    Vec3 swing_start;
    Vec3 swing_target;

    Vec3 stance_start;
    Vec3 stance_target;

	Vec3 offset;
	Vec3 offset_cmd;
};

struct VelState {
	Vec3 stance_vel = Vec3{0.0f, 0.0f, 0.0f};
	Vec3 step_vec = Vec3{Movement::STEP_LENGTH_MM, 0.0f, 0.0f};
};

class Leg {
	public:
		const float MPC_OFFSET_TAU_S = 0.08; // higher = slower response, more smoothing

		struct Config {
			int id;
		};

		Leg(const Config& cfg);

		void update(const float dt_s, Theta3& out_target_angles_deg, const ImuData& imu_data);
		void startSwing(const Vec3& target_pos_lf);
		void startStance(const Vec3& target_pos_lf);

		bool grounded() const;

		Vec3 nominalLandingPos() const;
		Vec3 currFootPos() const;
		Vec3 rearFootPos() const;
		float swingTimeRemainingS() const;

		void setMpcOffset(const Vec3& offset_lf);
		void setStance(const Vec3& vel, const Vec3& step);

		void resumeStance();
		void hold();

		void applyPidCorrection(const ImuData& imu_data);

	private:
		void moveLeg();

		void setState(const LegState state);

		void updateSwing(const float dt_s);
		void updateStance(const float dt_s);

		Theta3 angles; // deg
		float swing_phase_;
		float stance_phase_;

		int id_;
		LegState state_;
		FootState foot_state_;
		VelState vel_state_;
};

#endif