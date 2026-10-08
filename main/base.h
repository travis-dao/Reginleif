#ifndef BASE_h
#define BASE_h

#include "helpers.h"
#include "leg.h"
#include "pca9685.h"
// #include <pca9685.h>
#include <mpu6050.h>

// I2C config
namespace I2C_Config {
	constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
	constexpr gpio_num_t SDA_GPIO = GPIO_NUM_3;
	constexpr gpio_num_t SCL_GPIO = GPIO_NUM_4;
}

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

constexpr Vec3 body_offset = { 33.72f, 43.13f, 0.0f };

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

		Mat3 rot_mat;

    	i2c_master_bus_handle_t bus;
		mpu6050_angles_t angles;

		void init_i2c();

		/**
		* @brief Initializes the I2C bus and PCA9685 PWM/servo driver.
		*/
		void init_servo_driver();

		/**
		* @brief Allocates and initializes all four robot legs.
		*/
		void init_legs();

		/**
		* @brief Initializes and configures the MPU6050 IMU sensor.
		*/
		void init_imu();

		/**
		* @brief Sends initial calibration pulses to a single set of servo channels.
		*/
		void calibrate_servos();

		/**
		* @brief Updates movement state based on user/controller input.
		*
		* @param input The desired movement direction/magnitude vector.
		*/
		void input_controller(Vec3 input);

		/**
		* @brief Updates the internal state of all four legs.
		*/
		void update_legs();

		/**
		* @brief Reads sensor data from the IMU and updates the current orientation.
		*/
		void update_imu();

		/**
		* @brief Smoothly updates the current movement speed toward a target speed.
		*/
		void update_speed();

		/**
		* @brief Computes a target body orientation based on the currently airborne leg.
		*/
		void update_orientation();

		/**
		* @brief Calculates and updates rotation matrix for PID controller
		*/
		void update_rot_matrix(float delta_t);

		/**
		* @brief Advances the walking gait by selecting and moving the next leg.
		*/
		void move();

	public:
		static pca9685_handle_t pca;
		static mpu6050_handle_t mpu;
		

		/**
		* @brief Constructs a Base object with default/neutral state.
		*/
		Base();

		/**
		* @brief Performs full hardware and subsystem initialization for the robot base.
		*/
		void init();


		/**
		* @brief Runs one full update cycle for the robot base.
		*
		* @param dt_s Elapsed time in seconds since the last update call.
		*/
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


		/**
		* @brief Retrieves the current IMU-derived orientation angles.
		*
		* @return Vec3 containing roll and pitch from the filtered IMU angle
		*         data, with the z component unused (set to 0).
		*/
		Vec3 get_imu_angles();

		Vec3 get_target_orientation() {
			return this->target_orientation;
		}
		Mat3 get_rot_matrix() {
			return this->rot_mat;
		}
};

extern Base base;

#endif