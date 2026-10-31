#ifndef BASE_h
#define BASE_h

#include "leg_controller.h"
#include "rtos_utils.h"
#include "types.h"

class Base {
	public:
		enum MoveState {
			REST, WALK, RUN, TURN
		};

		Base();
		void init(i2c_master_bus_handle_t bus_);
		void update(const float dt_s, SharedData& shared_data);
		
		float getSpeed() { return this->current_speed_mm_s; }
		Vec3 getTargetOrientation() { return this->target_orientation_rad; }

	private:
		float current_speed_mm_s; // mm/s
		Vec3 target_orientation_rad; // rad

		MoveState state;
		LegController leg_controller_;
		Vec3 input;

		void updateInput(Vec3 input);
		void updateSpeed(const float dt_s);
};

#endif