#ifndef MOTION_SUBSYSTEM_h
#define MOTION_SUBSYSTEM_h

#include "eigen.h"
#include "frame.h"
#include "types.h"

#include "mpc.h"
#include "srb_model.h"
#include "state_estimator.h"

#include "leg_controller.h"

using Contact = Vector<Array<bool, NUM_LEGS>>;
using Leg3 = Array<Vec3f, NUM_LEGS>;   // one 3-vector per leg (FL, FR, BR, BL)
using FootRel = Leg3;

class MotionSubsystem {
    public:
        struct Config {
            MPC::Config& mpc_cfg;
            StateEstimator::Config& state_estimator_cfg;
            LegController& leg_controller_;

            size_t horizon = 10;

            float z_des_m = 0.0f;
            float vx_des_mps = Leg::stanceSpeed() * MM_TO_M;
            Optional<float> yaw_des = std::nullopt;

            float solve_period_s = 0.03f;
            float gain_mm_per_n = 1.0f;
            float max_correction_mm = 12.0f;
            float ramp_s = 2.0f;
        };

        MotionSubsystem(const Config& cfg);
        const MPCResult* step(const float dt_s, const ImuData& imu);
        void update(const float dt_s, const ImuData& imu);
    
    private:
        Vector<StateVec> reference(const State& s0) const;
        Contact contactSchedule() const;
        Vector<FootRel> footRelPos(const State& s0, Vector<StateVec>& x_ref, const Contact contact);
        Array<Vec3, NUM_LEGS> toOffsets(const FootRel& forces, const Array<bool, NUM_LEGS>& contact0, float yaw) const;
        FootRel footPositionsWorld(const State& s0) const;


        MPC mpc_;
        LegController& leg_controller_;
        StateEstimator state_estimator_;
        Config cfg_;

        float time_engaged_ = 0.0f;
        float time_since_ = 1e-9f;

        Array<Vec3, 4> offsets_;
        Optional<MPCResult> lastResult_;
};

#endif