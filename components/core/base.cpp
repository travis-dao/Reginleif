#include <algorithm>
#include <math.h>
#include <new>

#include "esp_log.h"
#include "freertos/projdefs.h"
#include "rtos_utils.h"
#include "types.h"

#include "base.h"

float SPEED_LERP_RATE = 4.0f;

Base::Base() {
	this->current_speed_mm_s = 0.0f;
	this->state = REST;
	this->target_orientation_rad = Vec3{0.0f, 0.0f, 0.0f};
}

esp_err_t Base::init(i2c_master_bus_handle_t bus) {
    esp_err_t err = this->leg_controller_.init(bus);

    if (err != ESP_OK) {
        return err;
    }

	return ESP_OK;
}

void Base::updateInput(Vec3 input) {
	this->input = input;
	
	state = this->input.x > 0.0f ? WALK : REST;
}

void Base::updateSpeed(const float dt_s) {
	float target_speed_mm_s;

	switch (this->state) {
	case REST:
		target_speed_mm_s = 0.0f;
		break;
	case WALK:
	case RUN:
		target_speed_mm_s =
			(Movement::k_step_length_mm * 2) /
			(Movement::k_swing_duration_s * 3); // calc stance speed
		break;
	default:
		target_speed_mm_s = 0.0f;
		break;
	}

	// higher SPEED_LERP_RATE = snappier response, lower = smoother/slower
	float t = 1.0f - expf(-SPEED_LERP_RATE * dt_s);
	t = std::clamp(t, 0.0f, 1.0f);

	this->current_speed_mm_s = std::lerp(this->current_speed_mm_s, target_speed_mm_s, t);

	// snap to zero to avoid tiny residual velocity keeping legs "moving"
	if (fabsf(this->current_speed_mm_s) < 0.001f) {
		this->current_speed_mm_s = 0.0f;
	}
}

void Base::update(const float dt_s, SharedData& shared_data) {
	shared_data.current_speed_mm_s = this->current_speed_mm_s;
	this->target_orientation_rad = shared_data.angles_rad;

	updateSpeed(dt_s);

	this->leg_controller_.update(dt_s);
}