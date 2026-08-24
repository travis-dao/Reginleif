#include <string.h>
#include <math.h>
#include <algorithm>

#include <pca9685.h>
#include <mpu6050.h>

#include "leg.h"
#include "helpers.h"
#include "base.h"

constexpr uint16_t MPU6050_I2C_ADDR = 0x68;

i2c_dev_t Base::pca = { };
mpu6050_handle_t Base::mpu = nullptr;

Base::Base() {
	this->velocity = Vec3 {0.0f, 0.0f, 0.0f};
	this->current_airborne_leg = -1;
	this->current_speed = 0.0f;
	this->state = REST;
	this->target_orientation = Vec3 { 0.0f, 0.0f, 0.0f };
}

void Base::init() {
	// init servos
	ESP_ERROR_CHECK(i2cdev_init());
	memset(&pca, 0, sizeof(i2c_dev_t));
	ESP_ERROR_CHECK(pca9685_init_desc(&pca, PCA9685_ADDR_BASE, I2C_PORT, SDA_GPIO, SCL_GPIO));
	ESP_ERROR_CHECK(pca9685_init(&pca));
	ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&pca, ServoConfig::SERVO_FREQ));

	printf("Driver set up.\n");
	ThisThread::sleep_for(1000ms);

	for(int i = 0; i < 4; i++) {
		this->legs[i] = new Leg(i);
	}

	printf("Initialized legs to neutral position.\n");
	ThisThread::sleep_for(1000ms);

	// init IMU
	mpu = mpu6050_create(I2C_PORT, MPU6050_I2C_ADDR);

	uint8_t device_id = 0;
	esp_err_t ret = mpu6050_get_deviceid(mpu, &device_id);
	while (ret != ESP_OK) {
		printf("Unable to connect to MPU.\n");
		ThisThread::sleep_for(500ms);
		ret = mpu6050_get_deviceid(mpu, &device_id);
	}

	ESP_ERROR_CHECK(mpu6050_config(mpu, ACCE_FS_4G, GYRO_FS_500DPS));
	ESP_ERROR_CHECK(mpu6050_wake_up(mpu));

	printf("Do not move MPU6050 during startup.\n");
	ThisThread::sleep_for(1000ms);

	// Note: this driver has no built-in offset calibration like calcOffsets();
	// the complementary filter's first call seeds roll/pitch from the
	// accelerometer only, so keep the robot still through that first update.
	this->current_orientation = get_imu_angles();

	printf("IMU ready.\n");
	ThisThread::sleep_for(1000ms);
}

void Base::update_legs() {
	for (int i = 0; i < 4; i++) {
		legs[i]->update();
	}
}

void Base::update_imu() {
	mpu6050_acce_value_t acce;
	mpu6050_gyro_value_t gyro;

	esp_err_t ret = mpu6050_get_acce(mpu, &acce);
	if (ret != ESP_OK) {
		printf("Failed to read accelerometer.\n");
		return;
	}

	ret = mpu6050_get_gyro(mpu, &gyro);
	if (ret != ESP_OK) {
		printf("Failed to read gyroscope.\n");
		return;
	}

	mpu6050_complimentory_filter(mpu, &acce, &gyro, &imu_angle);
	this->current_orientation = get_imu_angles();
}

void Base::move() {
	if (state == REST) {
		return;
	}

	// check if all legs are grounded
	bool all_legs_grounded = true;
	for (int i = 0; i < 4; i++) {
		if (!legs[i]->is_grounded()) {
			all_legs_grounded = false;
		}
	}

	// find next leg to move
	if (all_legs_grounded) {
		this->current_airborne_leg = get_new_leg(this->current_airborne_leg);
		this->legs[current_airborne_leg]->update_state(SWING);
	}
}

void Base::input_controller(Vec3 input) {
	this->input = input;

	if (input.magnitude() == 0) {
		state = REST;
	} else {
		state = WALK;
	}
}

void Base::update_speed() {
	float target_speed;

	switch (this->state) {
		case REST:
			target_speed = 0.0f;
			break;
		case WALK:
		case RUN:
			target_speed = (MovementConfig::MAX_STEP_LENGTH_MM * 2) / (MovementConfig::SWING_DURATION_S * 3);
			break;
		default:
			target_speed = 0.0f;
			break;
	}

	// higher SPEED_LERP_RATE = snappier response, lower = smoother/slower
	float t = 1.0f - expf(-SPEED_LERP_RATE * this->dt_s);
	t = std::clamp(t, 0.0f, 1.0f);

	this->current_speed = std::lerp(this->current_speed, target_speed, t);

	// snap to zero to avoid tiny residual velocity keeping legs "moving"
	if (fabs(this->current_speed) < 0.001f) {
		this->current_speed = 0.0f;
	}
}

// void Base::update_velocity() {
//   float magnitude = sqrt(this->input.x * this->input.x + this->input.y * this->input.y);
//
//   if (magnitude > 0.0) {
//     this->velocity = this->input / magnitude * this->current_speed;
//   } else {
//     this->velocity = { 0, 0, 0 };
//   }
// }

void Base::update(unsigned long dt_ms) {
	this->dt_s = dt_ms / 1000.0f;

	input_controller(Vec3 {1.0f, 0.0f, 0.0f});

	update_speed();
	// update_orientation();

	move();

	update_legs();
}

void Base::update_orientation() {
	float a = 10.0f;

	switch (this->current_airborne_leg) {
		case 0:
			target_orientation.x = a;
			target_orientation.y = -a;
			break;
		case 1:
			target_orientation.x = -a;
			target_orientation.y = a;
			break;
		case 2:
			target_orientation.x = -a;
			target_orientation.y = a;
			break;
		case 3:
			target_orientation.x = a;
			target_orientation.y = a;
			break;
		default:
			target_orientation.x = 0.0f;
			target_orientation.y = 0.0f;
			break;
	}
}

Vec3 Base::get_imu_angles() {
	return Vec3 {imu_angle.roll, imu_angle.pitch, 0.0f};
}