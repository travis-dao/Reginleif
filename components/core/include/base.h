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
		
		float getSpeed() { return this->current_speed; }
		float get_dt_s() { return this->dt_s; }
		int getCurrentAirborneLeg() { return this->current_airborne_leg; }

		Vec3 getImuAngles() { return this->current_orientation; }
		Vec3 getTargetOrientation() { return this->target_orientation; }

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

		void initI2C();
		void initServoDriver();
		void initImu();

		void initLegs();
		void calibrateServos();

		void inputController(Vec3 input);

		void updateLegs();
		void updateImu();
		void updateSpeed();

		void move();
};

#endif