#ifndef MOTION_SUBSYSTEM_h
#define MOTION_SUBSYSTEM_h

#include "types.h"
#include "motion_config.h"
#include "leg_controller.h"
#include "policy.h"   // PolicyNet, PolicyObservation (135x1), PolicyAction (12x1)
#include "imu_sample.h"

struct Command {
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;   // body frame, m/s, m/s, rad/s
};

class MotionSubsystem {
    public:
        enum Mode { INIT, STANDING_UP, RUNNING, FAULT };

        explicit MotionSubsystem(LegController& lc);

        // Call every control loop iteration with the real dt.
        void update(const float dt_s, const ImuSample& imu);
        void setCommand(float vx, float vy, float wz);
        Mode mode() const { return mode_; }

    private:
        void pushFrame(const ImuSample& imu);
        void startRunning(const ImuSample& imu);
        bool policyStep(const float dt_s, const ImuSample& imu);
        bool safetyOk(const ImuSample& imu) const;

        LegController& leg_controller_;
        PolicyNet policy_;
        Mode mode_ = INIT;
        Command cmd_;

        // ring buffer; head_ points at the OLDEST frame
        Array<Array<float, OBS_FRAME_DIM>, OBS_HISTORY> history_{};
        int head_ = 0;

        PolicyAction prev_action_;
        Array<float, NUM_JOINTS> q_target_{};
        Array<float, NUM_JOINTS> q_prev_tick_{};
        float t_since_ = 0.0f;
        float t_run_ = 0.0f;
};

#endif