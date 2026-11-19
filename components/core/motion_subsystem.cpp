#include "motion_subsystem.h"
#include <cmath>

MotionSubsystem::MotionSubsystem(LegController& lc) : leg_controller_(lc), policy_() {
    prev_action_.setZero();
}

void MotionSubsystem::setCommand(float vx, float vy, float wz) {
    cmd_.vx = vx;
    cmd_.vy = vy;
    cmd_.wz = wz;
}

void MotionSubsystem::update(const float dt_s, const ImuSample& imu) {
    if (leg_controller_.faulted()) mode_ = FAULT;

    switch (mode_) {
    case INIT:
        mode_ = STANDING_UP;
        break;

    case STANDING_UP:
        if (leg_controller_.moveToDefault(dt_s, STANDUP_RAD_S) && imu.valid) {
            startRunning(imu);
        }
        break;

    case RUNNING:
        if (!imu.valid || !safetyOk(imu)) {
            leg_controller_.estop();
            mode_ = FAULT;
            printf("Motion fault: tilt limit or invalid IMU, e-stop\n");
            break;
        }
        t_run_ += dt_s;
        policyStep(dt_s, imu);
        break;

    case FAULT:
        break;
    }
}

void MotionSubsystem::startRunning(const ImuSample& imu) {
    t_run_ = 0.0f;
    t_since_ = 0.0f;
    prev_action_.setZero();
    q_target_ = leg_controller_.defaultPos();
    q_prev_tick_ = leg_controller_.jointPos();

    // Prime the whole history with the current frame (matches env reset behaviour:
    // the env must also fill history with the first frame, not zeros).
    pushFrame(imu);
    const auto first = history_[(head_ + OBS_HISTORY - 1) % OBS_HISTORY];
    for (auto& f : history_) f = first;

    mode_ = RUNNING;
}

bool MotionSubsystem::safetyOk(const ImuSample& imu) const {
    // gravity_body.z = -1 when upright
    return imu.gravityBody().z() < -cosf(MAX_TILT_RAD);
}

void MotionSubsystem::pushFrame(const ImuSample& imu) {
    Array<float, OBS_FRAME_DIM> f;
    int i = 0;

    const Vec3 g = imu.gravityBody();
    for (int k = 0; k < 3; k++) f[i++] = imu.gyro_body[k] * GYRO_SCALE;
    for (int k = 0; k < 3; k++) f[i++] = g[k];

    f[i++] = cmd_.vx * CMD_SCALE_LIN;
    f[i++] = cmd_.vy * CMD_SCALE_LIN;
    f[i++] = cmd_.wz * CMD_SCALE_ANG;

    const auto& q = leg_controller_.jointPos();
    const auto& q0 = leg_controller_.defaultPos();
    for (int k = 0; k < NUM_JOINTS; k++) f[i++] = q[k] - q0[k];

    // joint velocity: commanded joint motion over the last policy period
    for (int k = 0; k < NUM_JOINTS; k++) {
        f[i++] = ((q[k] - q_prev_tick_[k]) / POLICY_PERIOD_S) * QD_SCALE;
    }
    q_prev_tick_ = q;

    for (int k = 0; k < NUM_JOINTS; k++) f[i++] = prev_action_[k];
    // i == OBS_FRAME_DIM

    history_[head_] = f;               // overwrite oldest
    head_ = (head_ + 1) % OBS_HISTORY; // head_ is now the new oldest
}

bool MotionSubsystem::policyStep(const float dt_s, const ImuSample& imu) {
    t_since_ += dt_s;

    if (t_since_ < POLICY_PERIOD_S) {
        // keep the slew limiter moving toward the last target between policy ticks
        leg_controller_.setJointTargets(dt_s, q_target_);
        return false;
    }
    t_since_ -= POLICY_PERIOD_S;
    if (t_since_ > POLICY_PERIOD_S) t_since_ = 0.0f;  // don't burst after a long stall

    pushFrame(imu);

    PolicyObservation obs;
    int n = 0;
    for (int h = 0; h < OBS_HISTORY; h++) {                 // oldest -> newest
        const auto& f = history_[(head_ + h) % OBS_HISTORY];
        for (int k = 0; k < OBS_FRAME_DIM; k++) obs[n++] = f[k];
    }

    PolicyAction act = policy_.infer(obs);                  // normalization inside policy_
    if (!act.allFinite()) {
        printf("Policy output non-finite, using zero action\n");
        act.setZero();
    }
    act = act.cwiseMax(-1.0f).cwiseMin(1.0f);

    const float ramp = fminf(1.0f, t_run_ / STARTUP_RAMP_S);
    const auto& q0 = leg_controller_.defaultPos();
    for (int i = 0; i < NUM_JOINTS; i++) {
        q_target_[i] = q0[i] + ACTION_SCALE_RAD * ramp * act[i];
    }

    prev_action_ = act;   // clamped, un-ramped (training must do the same)
    leg_controller_.setJointTargets(dt_s, q_target_);
    return true;
}