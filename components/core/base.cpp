#include "base.h"

float SPEED_LERP_RATE = 4.0f;

Base::Base(const Config& cfg)
	: motion_subsystem_(cfg.motion_subsystem_cfg), 
	  leg_controller_(),
	  cfg_(cfg) 
{
	current_speed_mm_s_ = 0.0f;
	state_ = REST;
	target_orientation_rad_ = Vec3{0.0f, 0.0f, 0.0f};
}

void Base::init(i2c_master_bus_handle_t bus) {
    leg_controller_.init(bus);
}

void Base::updateInput(const Vec3 input) {
	input_ = input;
	state_ = input_.x > 0.0f ? WALK : REST;
}

void Base::updateSpeed(const float dt_s) {
	float target_speed_mm_s;

	switch (state_) {
	case REST:
		target_speed_mm_s = 0.0f;
		break;
	case WALK:
	case RUN:
		target_speed_mm_s =
			(Movement::STEP_LENGTH_MM * 2) /
			(Movement::SWING_DURATION_S * 3); // calc stance speed
		break;
	default:
		target_speed_mm_s = 0.0f;
		break;
	}

	// higher SPEED_LERP_RATE = snappier response, lower = smoother/slower
	float t = 1.0f - expf(-SPEED_LERP_RATE * dt_s);
	t = std::clamp(t, 0.0f, 1.0f);

	current_speed_mm_s_ = std::lerp(current_speed_mm_s_, target_speed_mm_s, t);

	// snap to zero to avoid tiny residual velocity keeping legs "moving"
	if (fabsf(current_speed_mm_s_) < 0.001f) {
		current_speed_mm_s_ = 0.0f;
	}
}

void Base::update(const float dt_s, const ImuData& imu_data) {
	target_orientation_rad_ = imu_data.angles_rad * RAD_TO_DEG;

	updateSpeed(dt_s);
	
	motion_subsystem_.update(dt_s, imu_data);
}