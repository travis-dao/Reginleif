#ifndef BASE_h
#define BASE_h

#include "leg_controller.h"

class Base {
	public:
		enum MoveState {
			REST, WALK, RUN, TURN
		};

		Base();
		esp_err_t init(i2c_master_bus_handle_t bus_);
		void update(const float dt_s, SharedData& shared_data);
		
		float getSpeed() { return this->current_speed; }
		Vec3 getTargetOrientation() { return this->target_orientation; }

	private:
		float current_speed;
		Vec3 target_orientation = Vec3 { 0.0f, 0.0f, 0.0f };

		MoveState state;
		LegController leg_controller_;
		Vec3 input;

		void updateInput(Vec3 input);
		void updateSpeed(const float dt_s);
};

#endif