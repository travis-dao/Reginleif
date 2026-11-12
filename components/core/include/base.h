#ifndef BASE_h
#define BASE_h

#include "leg_controller.h"
#include "motion_subsystem.h"
#include "rtos_utils.h"
#include "types.h"

class Base {
	public:
		struct Config {
			MotionSubsystem::Config& motion_subsystem_cfg;
            MPC::Config& mpc_cfg;
            StateEstimator::Config& state_estimator_cfg;
		};

		enum MoveState {
			REST, WALK, RUN, TURN
		};

		Base(const Config& cfg);
		void init(const i2c_master_bus_handle_t bus_);
		void update(const float dt_s, const ImuData& imu_data);

	private:
		float current_speed_mm_s_; // mm/s
		Vec3 target_orientation_rad_; // rad

		MoveState state_;
		MotionSubsystem motion_subsystem_;
		LegController leg_controller_;
		Vec3 input_;
		Config cfg_;

		void updateInput(const Vec3 input);
		void updateSpeed(const float dt_s);
};

#endif