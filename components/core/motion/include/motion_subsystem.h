#ifndef MOTION_SUBSYSTEM_h
#define MOTION_SUBSYSTEM_h

#include "eigen.h"
#include "frame.h"
#include "rtos_utils.h"
#include "types.h"

#include "mpc.h"
#include "srb_model.h"
#include "state_estimator.h"

#include "leg_controller.h"

using Contact = Vector<Array<bool, NUM_LEGS>>;
using Leg3 = Array<Vec3f, NUM_LEGS>;   // one 3-vector per leg (FL, FR, BR, BL)
using FootRel = Leg3;

struct Command {
    float vel_cmd_mps = 0.0f;
    float yaw_rate_cmd = 0.0f; // rad/s, + = left
    float yaw_leash_rad = 0.3f; // how far yaw_des can run ahead of measured yaw
};

class MotionSubsystem {
    public:
        enum Mode {
            STAND, // lc MoveState::IDLE
            WALK // lc MoveState::WALKING/STOPPING
        };

        struct Config {
            size_t horizon = 10;

            float z_des_m = 0.0f;
            Optional<float> yaw_des = std::nullopt;
            float yaw_leash_rad = 0.3f;

            float solve_period_s = 0.03f;
            float gain_mm_per_n = 1.0f;
            float max_correction_mm = 12.0f;
            float ramp_s = 2.0f;
        };

        MotionSubsystem(
            const Config& cfg, 
            const MPC::Config& mpc_cfg, 
            const StateEstimator::Config& state_estimator_cfg, 
            LegController& leg_controller_cfg
        );
        const MPCResult* step(const float dt_s, const ImuData& imu);
        void update(const float dt_s, const ImuData& imu);

        // init the estimator at a known pose (neutral)
        void reset(const Vec3f& p0, const Vec3f& v0, const ImuData& imu);
        State estimate(const float dt_s, const ImuData& imu);
    
    private:
        Vector<StateVec> reference(const State& s0) const;
        Contact contactSchedule() const;
        Vector<FootRel> footRelWorld(const State& s0, Vector<StateVec>& x_ref, const Contact contact);
        Array<Vec3, NUM_LEGS> footRelBody();
        Array<Vec3, NUM_LEGS> toOffsets(const FootRel& forces, const Array<bool, NUM_LEGS>& contact0, float yaw) const;
        void updateMode(const State& s0);

        MPC mpc_;
        LegController& leg_controller_; // reference, base holds instance
        StateEstimator state_estimator_;
        Config cfg_;
        Mode mode_;
        Command cmd_;
        State state_;

        bool mpc_active_;
        float t_engaged_ = 0.0f;
        float t_since = 1e-9f;
        Optional<Eigen::Vector2f> hold_xy_ = std::nullopt;
        float yaw_des_;

        Array<Vec3, 4> offsets_;
        Optional<MPCResult> lastResult_;
};

#endif