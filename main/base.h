#ifndef BASE_h
#define BASE_h

#include "helpers.h"
#include "leg.h"
#include <pca9685.h>
#include <mpu6050.h>

// I2C config
constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
constexpr gpio_num_t SDA_GPIO = GPIO_NUM_3;
constexpr gpio_num_t SCL_GPIO = GPIO_NUM_4;

namespace ServoConfig {
	constexpr float SERVO_FREQ = 50.0f;
	constexpr float SERVO_MIN_PULSE = 160.0f;
	constexpr float SERVO_MAX_PULSE = 560.0f;
}

constexpr int next_leg_gait[5] = {
	/* -1 -> */ 0,
	/*  0 -> */ 2,
	/*  1 -> */ 3,
	/*  2 -> */ 1,
	/*  3 -> */ 0,
};

constexpr Vec3 body_offset = {42.0f, 66.7f, 0.0f};

enum MoveState {
	REST, WALK, RUN, TURN
};

class Base {
	private:
		float current_speed;
		Vec3 velocity;
		Leg* legs[4];
		float dt_s;
		int current_airborne_leg;
		Vec3 current_orientation;
		Vec3 target_orientation = Vec3 { 0.0f, 0.0f, 0.0f };
		MoveState state;
		Vec3 input;

		complimentary_angle_t imu_angle = { 0.0f, 0.0f };

		void init_servo_driver();
		void init_legs();
		void init_imu();
		void calibrate_servos();

		void input_controller(Vec3 input);

		void update_legs();
		void update_imu();
		void update_speed();
		void update_velocity();
		void update_orientation();

		void move();

	public:
		static i2c_dev_t pca;
		static mpu6050_handle_t mpu;
		
		Base();
		void init();
		void update(float dt_s);
		
		void drive_servo(float dt_s, float idx, float min, float max);

		int get_new_leg(int leg) {
			return next_leg_gait[leg + 1];
		}
		Vec3 get_velocity() {
			return this->velocity;
		}
		float get_speed() {
			return this->current_speed;
		}
		float get_dt_s() {
			return this->dt_s;
		}
		int get_current_airborne_leg() {
			return this->current_airborne_leg;
		}
		Vec3 get_imu_angles();

		Vec3 get_target_orientation() {
			return this->target_orientation;
		}
};

extern Base base;

#endif