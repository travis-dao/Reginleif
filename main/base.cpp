#include <cstdio>
#include <string.h>
#include <math.h>
#include <algorithm>

#include <pca9685.h>
#include <mpu6050.h>

#include "leg.h"
#include "helpers.h"
#include "base.h"

constexpr uint16_t MPU6050_I2C_ADDR = 0x68;
constexpr uint16_t PCA9685_I2C_ADDR = 0x40;

i2c_dev_t Base::pca = { };
mpu6050_handle_t Base::mpu = nullptr;

float SPEED_LERP_RATE = 4.0f;

Base::Base() {
	this->velocity = Vec3 {0.0f, 0.0f, 0.0f};
	this->current_airborne_leg = -1;
	this->current_speed = 0.0f;
	this->state = REST;
	this->target_orientation = Vec3 { 0.0f, 0.0f, 0.0f };
}

void Base::init() {
	printf("Reginleif Initialization Sequence...\n");
	init_servo_driver();
	init_legs();
	// calibrate_servos();
	init_imu();
}

void Base::init_servo_driver() {
	ESP_ERROR_CHECK(i2cdev_init());
	memset(&pca, 0, sizeof(i2c_dev_t));
	ESP_ERROR_CHECK(pca9685_init_desc(&pca, PCA9685_I2C_ADDR, I2C_PORT, SDA_GPIO, SCL_GPIO));
	ESP_ERROR_CHECK(pca9685_init(&pca));
	ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&pca, ServoConfig::SERVO_FREQ));

	printf("Servo driver set up.\n");
	ThisThread::sleep_for(1000ms);
}

void Base::init_legs() {
	for (int i = 0; i < 4; i++) {
		this->legs[i] = new Leg(i);
	}

	printf("Initialized legs to neutral position.\n");
	ThisThread::sleep_for(1000ms);
}

void Base::calibrate_servos() {
	// pca9685_set_pwm_value(&pca, 3, angle_to_pulse(90)); // coxa
	// pca9685_set_pwm_value(&pca, 4, angle_to_pulse(0)); // femur
	// pca9685_set_pwm_value(&pca, 5, angle_to_pulse(180)); // tibia

	for (int i = 0; i < 4; i++) {
		pca9685_set_pwm_value(&pca, i * 3, angle_to_pulse(90)); // coxa
	}

}

void Base::drive_servo(float dt_s, float idx, float min, float max) {
    enum State { MOVE_TO_MAX, PAUSE_AT_MAX, MOVE_TO_MIN, PAUSE_AT_MIN };
    static State state = MOVE_TO_MAX;
    static float cur   = min;
    static float timer = 0.0f;

    constexpr float kMoveDuration  = 2.0f; // seconds for 0->180 (or 180->0)
    constexpr float kPauseDuration = 1.0f; // seconds paused at each end
    const float speed = (max - min) / kMoveDuration; // deg/s

    switch (state) {
        case MOVE_TO_MAX:
            cur += speed * dt_s;
            if (cur >= max) {
                cur = max;
                timer = 0.0f;
                state = PAUSE_AT_MAX;
            }
            break;

        case PAUSE_AT_MAX:
            timer += dt_s;
            if (timer >= kPauseDuration) {
                state = MOVE_TO_MIN;
            }
            break;

        case MOVE_TO_MIN:
            cur -= speed * dt_s;
            if (cur <= min) {
                cur = min;
                timer = 0.0f;
                state = PAUSE_AT_MIN;
            }
            break;

        case PAUSE_AT_MIN:
            timer += dt_s;
            if (timer >= kPauseDuration) {
                state = MOVE_TO_MAX;
            }
            break;
    }

    pca9685_set_pwm_value(&pca, 3, angle_to_pulse(cur));
}

void Base::init_imu() {
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

	printf("Do not move IMU during startup.\n");
	ThisThread::sleep_for(1000ms);

	this->current_orientation = get_imu_angles();

	printf("IMU calibrated and set up.\n");
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

void Base::update(float dt_s) {
	this->dt_s = dt_s;

	input_controller(Vec3 {0.0f, 0.0f, 0.0f});

	update_speed();
	// update_orientation();

	if (base.get_dt_s() > 0) {
		// update_rot_matrix(dt_s);
	}

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

void Base::update_rot_matrix(float delta_t) {
  const static Vec3 ref_angles { 0, 0, 0 };

  static Vec3 prev_error { 0, 0, 0 };
  static Vec3 integral_sum { 0, 0, 0 };

  // error
  Vec3 error = ref_angles - get_imu_angles();

  // pid stuff
  Vec3 proportional_term = error * PIDConfig::K_p;

  integral_sum = integral_sum + (error * delta_t);
//   integral_sum = clamp_vec3(integral_sum, -PIDConfig::INTEGRAL_LIMIT_DEG, PIDConfig::INTEGRAL_LIMIT_DEG); // anti-windup

  Vec3 integral_term = integral_sum * PIDConfig::K_i;

  Vec3 derivative_term = (error - prev_error) / delta_t * PIDConfig::K_d;
  // low pass filter, kalmin filter

  // add the P, I, D terms
  Vec3 compensation_angles_deg = proportional_term + integral_term + derivative_term;
//   compensation_angles_deg = clamp_vec3(compensation_angles_deg, -PIDConfig::COMPENSATION_LIMIT_DEG, PIDConfig::COMPENSATION_LIMIT_DEG); // output saturation

  Vec3 compensation_angles = compensation_angles_deg * (M_PI / 180);

  // rotation matrix
  Mat3 rotation_matrix_x {{
    {1, 								0, 							0},
    {0, cos(compensation_angles.x), -sin(compensation_angles.x)},
    {0, sin(compensation_angles.x), cos(compensation_angles.x)}
  }};
  Mat3 rotation_matrix_y {{
    {cos(compensation_angles.y),  0, sin(compensation_angles.y)},
    {0, 								1, 							 0},
    {-sin(compensation_angles.y), 0, cos(compensation_angles.y)}
  }};
  Mat3 rotation_matrix_z {{
    {cos(compensation_angles.z), -sin(compensation_angles.z), 0},
    {sin(compensation_angles.z), cos(compensation_angles.z),  0},
    {0, 							   0, 							 1}
  }};

  Mat3 rotation_matrix = rotation_matrix_x * rotation_matrix_y * rotation_matrix_z;

  prev_error = error;

  this->rot_mat = rotation_matrix;
}

Vec3 Base::get_imu_angles() {
	return Vec3 {imu_angle.roll, imu_angle.pitch, 0.0f};
}