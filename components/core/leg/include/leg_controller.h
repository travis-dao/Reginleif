#ifndef LEG_CONTROLLER_h
#define LEG_CONTROLLER_h

#include "leg.h"
#include "pca9685.h"
#include "rtos_utils.h"
#include "leg_config.h"

class LegController {
    public:
        LegController();

        void update(const float dt_s, const ImuData& imu_data);
        void init(i2c_master_bus_handle_t bus);
        
        bool isGrounded() { return this->grounded; }
		int getSwingLeg() { return this->swing_leg; }

        Array<Vec3, 4> footPositionsBody() const;
        Array<Vec3, 4> landingTargetsBody() const;

    private:
        Leg* legs_[4];
		Pca9685 servo_driver_;

        Array<Theta3, 4> target_angles;
        float target_angles_buffer[12];

		int swing_leg;
        bool grounded;
};

#endif