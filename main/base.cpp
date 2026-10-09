#include <cstdio>
#include <string.h>
#include <math.h>
#include <algorithm>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "pca9685.h"

#include "leg.h"
#include "helpers.h"
#include "base.h"
#include "config.h"

float SPEED_LERP_RATE = 4.0f;

Base::Base() {
	this->current_airborne_leg = -1;
	this->current_speed = 0.0f;
	this->state = REST;
	this->target_orientation = Vec3 { 0.0f, 0.0f, 0.0f };
}

void Base::init() {
	printf("Initialization Sequence\n");
	init_i2c();

	// ping connected devices
	// for (uint8_t addr = 0x03; addr < 0x78; addr++) {
	// 	if (i2c_master_probe(this->bus, addr, 100) == ESP_OK) {
	// 		printf("Found device at 0x%02X\n", addr);
	// 	}
	// }

	init_servo_driver();
	init_legs();
	// calibrate_servos();

	init_imu();
}

void Base::init_i2c() {
	// init i2c bus
	esp_log_level_set("gpio", ESP_LOG_WARN); // disable warnings
	printf("Initializing I2C .......... ");
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port          = Config::I2C::I2C_PORT;
    bus_cfg.sda_io_num        = Config::I2C::SDA_GPIO;
    bus_cfg.scl_io_num        = Config::I2C::SCL_GPIO;
    bus_cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

	ThisThread::sleep_for(1000ms);
	printf("Success\n");
}

void Base::init_servo_driver() {
	// init servo driver and connect to master i2c bus
	printf("Initializing PCA9685 .......... ");
	pca9685_config_t cfg = {};
    cfg.bus         = this->bus;
    cfg.addr        = PCA9685_I2C_ADDR_DEFAULT;
    cfg.pwm_freq_hz = ServoConfig::SERVO_FREQ;
    ESP_ERROR_CHECK(pca9685_new(&cfg, &this->pca));

	ThisThread::sleep_for(1000ms);
	printf("Success\n");
}

void Base::init_legs() {
	// instantiate legs
	printf("Initializing legs to neutral positions .......... ");
	for (int i = 0; i < 4; i++) {
		this->legs[i] = new Leg(i);
	}

	ThisThread::sleep_for(1000ms);
	printf("Success\n");
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

void Base::init_imu() {
	// init imu and connect to master i2c bus
	esp_log_level_set("mpu6050", ESP_LOG_ERROR);
	printf("Initializing MPU6050 .......... ");
    mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT();
    cfg.bus = this->bus;
    ESP_ERROR_CHECK(mpu6050_init(&cfg, &this->mpu));
	printf("Success\n");

	// calibrating imu
	ThisThread::sleep_for(500ms);
	printf("Calibrating MPU6050 .......... ");

    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(this->mpu, 200));
    ESP_ERROR_CHECK(mpu6050_calibrate_level(this->mpu, 200));

	this->current_orientation = get_imu_angles();

	ThisThread::sleep_for(1000ms);
	printf("Success\n");
}

void Base::update_legs() {
	// call update on each leg
	for (int i = 0; i < 4; i++) {
		legs[i]->update();
	}
}

void Base::update_imu() {
	// try to update imu with new measurements
	if (mpu6050_get_angles(this->mpu, &this->angles) != ESP_OK) {
		printf("Failed to read IMU\n");
		return;
	}

	// update current orientation -- maybe remove in the future
	this->current_orientation.x = -angles.roll;
	this->current_orientation.y = -angles.pitch;
	this->current_orientation.z = 0.0f;
	
	printf("roll: %.2f | pitch: %.2f\n", current_orientation.x, current_orientation.y);
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
		this->current_airborne_leg = Config::Movement::next_leg_gait[this->current_airborne_leg + 1];
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
			target_speed = (Config::Movement::MAX_STEP_LENGTH_MM * 2) / (Config::Movement::SWING_DURATION_S * 3); // calc stance speed
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

void Base::update_rot_matrix(float delta_t) {
  const static Vec3 ref_angles { 0, 0, 0 };
  static Vec3 prev_error { 0, 0, 0 };
  static Vec3 integral_sum { 0, 0, 0 };

  // error
  Vec3 error = ref_angles - get_imu_angles();

  // pid stuff
  Vec3 proportional_term = error * Config::PID::K_p;

  integral_sum = integral_sum + (error * delta_t);
//   integral_sum = clamp_vec3(integral_sum, -Config::PID::INTEGRAL_LIMIT_DEG, Config::PID::INTEGRAL_LIMIT_DEG); // anti-windup

  Vec3 integral_term = integral_sum * Config::PID::K_i;

  Vec3 derivative_term = (error - prev_error) / delta_t * Config::PID::K_d;

  // add the P, I, D terms
  Vec3 compensation_angles_deg = proportional_term + integral_term + derivative_term;
//   compensation_angles_deg = clamp_vec3(compensation_angles_deg, -Config::PID::COMPENSATION_LIMIT_DEG, Config::PID::COMPENSATION_LIMIT_DEG); // output saturation

  Vec3 compensation_angles = compensation_angles_deg * (M_PI / 180);

  // calculate rotation matrix
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


// void Base::update_orientation() {
// 	this->current_orientation = get_imu_angles();
// 	return;

// 	float a = 10.0f;

// 	switch (this->current_airborne_leg) {
// 		case 0:
// 			target_orientation.x = a;
// 			target_orientation.y = -a;
// 			break;
// 		case 1:
// 			target_orientation.x = -a;
// 			target_orientation.y = a;
// 			break;
// 		case 2:
// 			target_orientation.x = -a;
// 			target_orientation.y = a;
// 			break;
// 		case 3:
// 			target_orientation.x = a;
// 			target_orientation.y = a;
// 			break;
// 		default:
// 			target_orientation.x = 0.0f;
// 			target_orientation.y = 0.0f;
// 			break;
// 	}
// }