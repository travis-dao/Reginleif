#ifndef BASE_h
#define BASE_h

#include "leg.h"
#include "pca9685.h"
#include "mpu6050.h"
#include "ism330.h"
#include "pid_stabilize.h"

enum MoveState {
	REST, WALK, RUN, TURN
};

class Base {
	public:
		Base();
		void init();
		void update(float dt_s);
		
		float get_speed() { return this->current_speed; }
		float get_dt_s() { return this->dt_s; }
		int get_current_airborne_leg() { return this->current_airborne_leg; }

		Vec3 get_imu_angles() { return this->current_orientation; }
		Vec3 get_target_orientation() { return this->target_orientation; }

	private:
		float current_speed;
		int current_airborne_leg;
		Vec3 current_orientation;
		Vec3 target_orientation = Vec3 { 0.0f, 0.0f, 0.0f };

		MoveState state;
		Leg* legs_[4];

		float dt_s;
		Vec3 input;

    	i2c_master_bus_handle_t bus_;
		pca9685::Pca9685 servo_driver_;
		mpu6050::Mpu6050 imu_;
		mpu6050::Angles angles;

		Stabilizer stabilizer_;

		void init_i2c();
		void init_servo_driver();
		void init_imu();

		void init_legs();
		void calibrate_servos();

		void input_controller(Vec3 input);

		void update_legs();
		void update_imu();
		void update_speed();

		void move();
};

#endif