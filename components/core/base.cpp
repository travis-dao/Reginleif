#include <algorithm>
#include <math.h>

#include "esp_log.h"
#include "freertos/projdefs.h"

#include "base.h"

float SPEED_LERP_RATE = 4.0f;

Base::Base() {
	this->current_speed = 0.0f;
	this->state = REST;
	this->target_orientation = Vec3{0.0f, 0.0f, 0.0f};
}

esp_err_t Base::init(i2c_master_bus_handle_t bus) {
    esp_err_t err = leg_controller_.init(bus);

    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

void Base::updateInput(Vec3 input) {
	this->input = input;

	if (input.magnitude() == 0) {
		state = REST;
	} else {
		state = WALK;
	}
}

void Base::updateSpeed(const float dt_s) {
	float target_speed;

	switch (this->state) {
	case REST:
		target_speed = 0.0f;
		break;
	case WALK:
	case RUN:
		target_speed =
			(Movement::MAX_STEP_LENGTH_MM * 2) /
			(Movement::SWING_DURATION_S * 3); // calc stance speed
		break;
	default:
		target_speed = 0.0f;
		break;
	}

	// higher SPEED_LERP_RATE = snappier response, lower = smoother/slower
	float t = 1.0f - expf(-SPEED_LERP_RATE * dt_s);
	t = std::clamp(t, 0.0f, 1.0f);

	this->current_speed = std::lerp(this->current_speed, target_speed, t);

	// snap to zero to avoid tiny residual velocity keeping legs "moving"
	if (fabsf(this->current_speed) < 0.001f) {
		this->current_speed = 0.0f;
	}
}

void Base::update(const float dt_s, SharedData& shared_data) {
	shared_data.current_speed = this->current_speed;

	updateSpeed(dt_s);

	leg_controller_.update(dt_s, shared_data);
}