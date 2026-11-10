#ifndef LEG_CONTROLLER_h
#define LEG_CONTROLLER_h

#include "leg.h"
#include "pca9685.h"
#include "rtos_utils.h"
#include "leg_config.h"
#include "eigen.h"

using SwingSchedule = Vector<Triple<int, float, float>>;

class LegController {
    public:
		enum MoveState {
			REST, WALK, RUN, TURN
		};

        LegController();

        void update(const float dt_s, const ImuData& imu_data);
        void init(i2c_master_bus_handle_t bus);
        
        bool grounded() const { return grounded_; }
		int swingLeg() const { return swing_leg_; }

        float swingTimeRemainingS() const {
            if (swing_leg_ < 0) {
                return 0.0f;
            } else {
                return legs_[swing_leg_]->swingTimeRemainingS();
            }
        }

        Array<Vec3, 4> footPositionsBody() const;
        Array<Vec3, 4> landingTargetsBody() const;

        void applyMpcCorrections(Array<Vec3, 4> offsets_lf);

        SwingSchedule swingSchedule(float horizon_s) const;

    private:
        Leg* legs_[4];
		Pca9685 servo_driver_;

        MoveState state_;

        Array<Theta3, 4> target_angles_;
        float target_angles_buffer_[12];

		int swing_leg_;
        int last_swing_leg_;
        float gap_remaining_s;
        bool grounded_;

        void startWalk();
};

#endif