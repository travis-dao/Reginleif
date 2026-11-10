#ifndef LEG_h
#define LEG_h

#include "leg_config.h"
#include "rtos_utils.h"
#include "types.h"

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

	Vec3 mpc_offset_lf;
	Vec3 mpc_offset_cmd;
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

		bool grounded() const { return state_ != SWING; }

		Vec3 nominalLandingPos() const;
		Vec3 currFootPos() const { return pos_state_.foot_curr_pos_lf; }
		Vec3 rearFootPos() const;
		float swingTimeRemainingS() const {
			if (state_ == LegState::SWING) {
				return 0.0f;
			} else {
				return (1.0f - swing_phase_) * Movement::SWING_DURATION_S;
			}
		}

		void setMpcOffset(const Vec3& offset_lf) { pos_state_.mpc_offset_cmd = offset_lf; }

		void applyPidCorrection(const ImuData& imu_data);

		// m/s
		static float stanceSpeed();

	private:
		Theta3 servo_angles_deg_;
		float swing_phase_;
		float stance_phase_;

		int id_;
		LegState state_;
		FootPositionState pos_state_;

		void moveLeg();

		void setState(const LegState state) { state_ = state; }
		void updateSwing(const float dt_s);
		void updateStance(const float dt_s);
};

#endif