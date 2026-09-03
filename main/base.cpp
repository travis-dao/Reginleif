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
constexpr uint16_t PCA9685_I2C_ADDR = PCA9685_ADDR_BASE; // 0x40

i2c_dev_t Base::pca = { };
mpu6050_handle_t Base::mpu = nullptr;

float SPEED_LERP_RATE = 4.0f;

/**
 * @brief Constructs a Base object with default/neutral state.
 *
 * Initializes velocity to zero, marks no leg as airborne (-1),
 * sets current speed to zero, sets the movement state to REST,
 * and zeroes out the target orientation.
 */
Base::Base() {
	this->velocity = Vec3 {0.0f, 0.0f, 0.0f};
	this->current_airborne_leg = -1;
	this->current_speed = 0.0f;
	this->state = REST;
	this->target_orientation = Vec3 { 0.0f, 0.0f, 0.0f };
}

/**
 * @brief Performs full hardware and subsystem initialization for the robot base.
 *
 * Initializes the servo driver, creates and initializes all legs, and
 * initializes and calibrates IMU.
 */
void Base::init() {
	printf("Reginleif Initialization Sequence...\n");
	init_servo_driver();
	init_legs();
	calibrate_servos();
	// init_imu();
}

/**
 * @brief Initializes the I2C bus and PCA9685 PWM/servo driver.
 *
 * Sets up the I2C descriptor for the PCA9685, configures the driver at
 * its I2C address, and sets the PWM frequency used for servo control.
 */
void Base::init_servo_driver() {
	ESP_ERROR_CHECK(i2cdev_init());
	memset(&pca, 0, sizeof(i2c_dev_t));
	ESP_ERROR_CHECK(pca9685_init_desc(&pca, PCA9685_I2C_ADDR, I2C_PORT, SDA_GPIO, SCL_GPIO));
	ESP_ERROR_CHECK(pca9685_init(&pca));
	ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&pca, ServoConfig::SERVO_FREQ));

	printf("Servo driver set up.\n");
	ThisThread::sleep_for(1000ms);
}

/**
 * @brief Allocates and initializes all four robot legs.
 *
 * Creates a new Leg object for each of the four legs (indices 0–3),
 * setting them to their neutral starting position.
 */
void Base::init_legs() {
	for (int i = 0; i < 4; i++) {
		this->legs[i] = new Leg(i);
	}

	printf("Initialized legs to neutral position.\n");
	ThisThread::sleep_for(1000ms);
}

/**
 * @brief Sends initial calibration pulses to a single set of servo channels.
 *
 * Sets the coxa, femur, and tibia servos to fixed
 * reference angles (X°, Y°, Z° respectively) for calibration purposes.
 */
void Base::calibrate_servos() {
	pca9685_set_pwm_value(&pca, 3, angle_to_pulse(90)); // coxa
	pca9685_set_pwm_value(&pca, 4, angle_to_pulse(0)); // femur
	pca9685_set_pwm_value(&pca, 5, angle_to_pulse(0)); // tibia
}

/**
 * @brief Initializes and configures the MPU6050 IMU sensor.
 *
 * Creates the IMU handle, repeatedly attempts to verify communication
 * with the device via its device ID, configures accelerometer/gyroscope
 * full-scale ranges, wakes the sensor, and captures an initial orientation
 * reading.
 */
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

/**
 * @brief Updates the internal state of all four legs.
 *
 * Calls update() on each Leg object, advancing their motion/state logic
 * for the current control cycle.
 */
void Base::update_legs() {
	for (int i = 0; i < 4; i++) {
		legs[i]->update();
	}
}

/**
 * @brief Reads sensor data from the IMU and updates the current orientation.
 *
 * Retrieves accelerometer and gyroscope readings from the MPU6050,
 * applies a complementary filter to combine them into a stable angle
 * estimate, and updates the base's current orientation accordingly.
 * Logs an error and returns early if either sensor read fails.
 */
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

/**
 * @brief Advances the walking gait by selecting and moving the next leg.
 *
 * Does nothing if the base is in the REST state. Otherwise, checks
 * whether all legs are currently grounded; if so, selects the next leg
 * to lift (based on the previously airborne leg) and transitions it
 * into the SWING state.
 */
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

/**
 * @brief Updates movement state based on user/controller input.
 *
 * Stores the given input vector and sets the base's state to REST if
 * the input has zero magnitude, or WALK otherwise.
 *
 * @param input The desired movement direction/magnitude vector.
 */
void Base::input_controller(Vec3 input) {
	this->input = input;

	if (input.magnitude() == 0) {
		state = REST;
	} else {
		state = WALK;
	}
}

/**
 * @brief Smoothly updates the current movement speed toward a target speed.
 *
 * Determines a target speed based on the current state (zero for REST,
 * a computed stride-based speed for WALK/RUN), then exponentially
 * interpolates the current speed toward that target using the elapsed
 * time step and SPEED_LERP_RATE. Snaps very small speeds to exactly
 * zero to prevent residual leg motion.
 */
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

/**
 * @brief Runs one full update cycle for the robot base.
 *
 * Stores the elapsed time step, applies a hardcoded forward input,
 * updates the current speed, updates the target body orientation, 
 * advances the gait/leg-swing logic, and updates all legs.
 *
 * @param dt_s Elapsed time in seconds since the last update call.
 */
void Base::update(float dt_s) {
	this->dt_s = dt_s;

	input_controller(Vec3 {1.0f, 0.0f, 0.0f});

	update_speed();
	// update_orientation();

	move();

	update_legs();
}

/**
 * @brief Computes a target body orientation based on the currently airborne leg.
 *
 * Sets target roll/pitch (x/y) offsets depending on which leg (0–3) is
 * currently swinging, used to shift body weight/balance during gait.
 * Defaults to a level (zero) orientation when no leg is airborne.
 */
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

/**
 * @brief Retrieves the current IMU-derived orientation angles.
 *
 * @return Vec3 containing roll and pitch from the filtered IMU angle
 *         data, with the z component unused (set to 0).
 */
Vec3 Base::get_imu_angles() {
	return Vec3 {imu_angle.roll, imu_angle.pitch, 0.0f};
}