#ifndef LEG_CONTROLLER_h
#define LEG_CONTROLLER_h

#include "leg.h"
#include "pca9685.h"
#include "rtos_utils.h"
#include "leg_config.h"
#include "eigen.h"
#include "types.h"
#include "srb_model.h"

// <leg, start, stop>
using SwingSchedule = Vector<Triple<int, float, float>>;

struct GaitConfig {
    float time_stance_s = 3.0f * MovementConfig::SWING_DURATION_S + 4.0f * MovementConfig::GAP_DURATION_S;
    float v_max_mm_s = MovementConfig::STEP_LENGTH_MM / time_stance_s;
    float max_step_y_mm = 20.0f; // mm, sideways reach -- TUNE
    float start_push_s = MovementConfig::SWING_DURATION_S;
};

struct GaitClock {
    float duty_zheta;
    float duty; // >=0.75 = no overlap, <0.75 = overlap
    float period_s;
    Array<float, 4> psi_; // internal phase clocks
    float grounded_s; // time since last 
};

struct MotionLimits { // slew-rate limiter ~ acceleration
    float a_v_mm_s2 = 150.0f; // max vel change
    float a_w_rad_s2 = 150.0f; // max omega change
};


struct Twist {
    float v = 0.0f; // mm/s
    float w = 0.0f; // rad/s
};
struct TwistState {

    Twist cmd; // request body motion
    Twist curr; // actual body twist
    Twist geom; // limited twist required to calc step geometry
};


class LegController {
    public:
		enum MoveState {
			IDLE, WALKING, STOPPING
		};

        LegController();

        void update(const float dt_s, const State& state, const ImuCorrection& imu_correction);
        void init(i2c_master_bus_handle_t bus);
        
        bool grounded() const;
        bool idle() const;
		int swingLeg() const;
        int nextSwingLeg() const;
        GaitConfig gaitConfig() const;

        float swingTimeRemainingS() const;

        Array<Vec3, 4> footPositionsBody() const;
        Array<Vec3, 4> landingTargetsBody();

        void applyMpcCorrections(Array<Vec3, 4> offsets_lf);

        SwingSchedule swingSchedule(float horizon_s) const;

        void setBodyCommand(const float v_mm_s, const float w_rad_s);
        void setBodyTilt(const float roll_rad, const float pitch_rad);
        Twist bodyTwist() const;

        void requestStart();
        void requestStop();

    private:
        void startWalk();
        void startSwing(const int leg);
        void setState(MoveState new_state);

        void updateTwist(const float dt_s);
        Gait updateGaitOrder();
        int mostOverdueLeg();

        Twist limit(Twist twist) const;

        void advanceClock(const float dt_s);
        void scheduleFrom(const int leg, const float delay_s);

        Vec3 nominalLandingPos(const int leg);

        Leg* legs_[4];
		Pca9685 servo_driver_;

        MoveState move_state_;
        const GaitConfig gait_cfg_;
        const MotionLimits limits_;
        Gait gait_;
        GaitClock gait_clock_;
        TwistState twist_;
        Array<Vec3, 4> nominal_leg_pos_bf_; // reference points for calculating rotation velocity during turning
        Vec3 vel_cmd_xy_;
        State state_;

        Array<Theta3, 4> target_angles_;
        float target_angles_buffer_[12];
        float body_roll_;
        float body_pitch_;

		int swing_leg_;
        int last_swing_leg_;
        bool grounded_;
};

#endif