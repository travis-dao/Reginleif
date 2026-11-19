#ifndef BASE_h
#define BASE_h

#include "leg_controller.h"
#include "motion_subsystem.h"
#include "imu_sample.h"
#include "types.h"

class Base {
	public:
		explicit Base();
		void init();
		void update(const float dt_s, const ImuSample& imu_sample);

	private:
		LegController leg_controller_;
		MotionSubsystem motion_subsystem_;
		Vec3 input_;

		void updateInput(const Vec3 input);
};

#endif