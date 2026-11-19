#include "base.h"

Base::Base() : leg_controller_(), motion_subsystem_(leg_controller_) { }

void Base::init() {
    leg_controller_.init();
}

void Base::updateInput(const Vec3 input) {
	input_ = input;
}

void Base::update(const float dt_s, const ImuSample& imu_sample) {
	updateInput(Vec3(0.0f, 0.0f, 0.0f));
	motion_subsystem_.update(dt_s, imu_sample);
}