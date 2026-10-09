#ifndef BASE_h
#define BASE_h

#include "helpers.h"
#include "leg.h"
#include "pca9685.h"
#include "mpu6050.h"

enum MoveState {
	REST, WALK, RUN, TURN
};

class Base {
	private:
		float current_speed;
		int current_airborne_leg;
		Vec3 current_orientation;
		Vec3 target_orientation = Vec3 { 0.0f, 0.0f, 0.0f };

		MoveState state;
		Leg* legs[4];

		float dt_s;
		Vec3 input;

		Mat3 rot_mat;

    	i2c_master_bus_handle_t bus;
		mpu6050_angles_t angles;

		void init_i2c();
		void init_servo_driver();
		void init_imu();

		void init_legs();


		void calibrate_servos();

		void input_controller(Vec3 input);

		void update_legs();
		void update_imu();
		void update_speed();
		void update_rot_matrix(float delta_t);

		void move();

	public:
		static pca9685_handle_t pca;
		static mpu6050_handle_t mpu;
		
		Base();
		void init();
		void update(float dt_s);
		
		float get_speed() { return this->current_speed; }
		float get_dt_s() { return this->dt_s; }
		int get_current_airborne_leg() { return this->current_airborne_leg; }

		Vec3 get_imu_angles();
		Vec3 get_target_orientation() { return this->target_orientation; }
		Mat3 get_rot_matrix() { return this->rot_mat; }
};

extern Base base;

#endif