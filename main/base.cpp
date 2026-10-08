#include <cstdio>
#include <string.h>
#include <math.h>
#include <algorithm>

#include "driver/i2c_master.h"
#include <mpu6050.h>
#include <pca9685.h>

#include "leg.h"
#include "helpers.h"
#include "base.h"

pca9685_handle_t Base::pca = nullptr;
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
	init_i2c();

	init_servo_driver();
	init_legs();
	// calibrate_servos();

	init_imu();
}

void Base::init_i2c() {
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port          = I2C_Config::I2C_PORT;
    bus_cfg.sda_io_num        = I2C_Config::SDA_GPIO;
    bus_cfg.scl_io_num        = I2C_Config::SCL_GPIO;
    bus_cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
}

void Base::init_servo_driver() {
	pca9685_config_t cfg = {};
    cfg.bus         = bus;
    cfg.addr        = PCA9685_I2C_ADDR_DEFAULT;
    cfg.pwm_freq_hz = ServoConfig::SERVO_FREQ;
	cfg.bus = bus;
    ESP_ERROR_CHECK(pca9685_new(&cfg, &this->pca));

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
	// Calibrate all servos on 1 leg
	// servo_set_angle(this->pca, 3, 90); // coxa
	// servo_set_angle(this->pca, 4, 0); // femur
	// servo_set_angle(this->pca, 5, 180); // tibia

	// Calibrate all servos of 1 leg section
	for (int i = 0; i < 4; i++) {
		servo_set_angle(this->pca, i * 3, 90);
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

    servo_set_angle(this->pca, 3, cur);
}

void Base::init_imu() {
    mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT();
    cfg.bus = bus;
    ESP_ERROR_CHECK(mpu6050_init(&cfg, &this->mpu));

	printf("Do not move IMU during startup.\n");
	ThisThread::sleep_for(1000ms);

    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(this->mpu, 200));

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
	if (mpu6050_get_angles(this->mpu, &this->angles) != ESP_OK) {
		printf("Failed to read IMU");
		return;
	}
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
	if (fabsf(this->current_speed) < 0.001f) {
		this->current_speed = 0.0f;
	}
}

void Base::update(float dt_s) {
	this->dt_s = dt_s;
	update_imu();

	input_controller(Vec3 {0.0f, 0.0f, 0.0f});

	update_speed();
	// update_orientation();

	if (base.get_dt_s() > 0) {
		// update_rot_matrix(dt_s);
	}

	move();

	// update_legs();
}

void Base::update_orientation() {
	this->current_orientation = get_imu_angles();
	return;

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
	return Vec3 {-angles.roll, -angles.pitch, 0.0f};
}