#include "leg_controller.h"
#include "leg_config.h"
#include "servo_config.h"
#include "ik.h"
#include "types.h"
#include <cmath>

// --------------- helpers (moved from leg.cpp) ---------------
static bool isRightLeg(const int id) { return id == 1 || id == 2; }

static Theta3 clampToLimits(const Theta3& angles_deg, const int leg_idx) {
    const LegServoLimits& lim = ServoConfig::limits[leg_idx];
    return Theta3 {
        lim.coxa.clamp(angles_deg.coxa),
        lim.femur.clamp(angles_deg.femur),
        lim.tibia.clamp(angles_deg.tibia)
    };
}

// leg-frame foot position -> IK-domain angles (deg). Same y flip as Leg::moveLeg.
static bool footToIkDeg(const int leg, Vec3 foot_lf, Theta3& out_deg) {
    foot_lf.y() *= isRightLeg(leg) ? -1.0f : 1.0f;
    return IK({
        .body_coxa_x_mm = BODY_COXA_X_MM,
        .femur_length_mm = FEMUR_LENGTH_MM,
        .tibia_length_mm = TIBIA_LENGTH_MM,
        .c_min = c_min,
        .c_max = c_max
    }, foot_lf, out_deg);
}

// IK-domain angles (deg) -> servo angles (deg): reverse-mounted inversion + limit clamp.
static Theta3 toServoDeg(const int leg, Theta3 ik_deg) {
    if (!isRightLeg(leg)) ik_deg = invertAngles(ik_deg);
    return clampToLimits(ik_deg, leg);
}

// --------------- init ---------------
void LegController::init() {
    driver_.init();

    for (int i = 0; i < NUM_LEGS; i++) {
        Theta3 th;
        if (!footToIkDeg(i, Offset::neutral_foot_pos_lf[i], th)) {
            faulted_ = true;   // neutral pose unreachable: geometry/config error
            return;
        }
        q_default_[3 * i + 0] = th.coxa  * DEG_TO_RAD;
        q_default_[3 * i + 1] = th.femur * DEG_TO_RAD;
        q_default_[3 * i + 2] = th.tibia * DEG_TO_RAD;
    }

    // Assume the robot starts at the neutral pose (as the old Leg constructor did).
    q_ = q_default_;
    writeServos();
}

// --------------- output ---------------
void LegController::writeServos() {
    for (int i = 0; i < NUM_LEGS; i++) {
        Theta3 ik_deg {
            q_[3 * i + 0] * RAD_TO_DEG,
            q_[3 * i + 1] * RAD_TO_DEG,
            q_[3 * i + 2] * RAD_TO_DEG
        };
        const Theta3 s = toServoDeg(i, ik_deg);
        buf_[3 * i + 0] = s.coxa;
        buf_[3 * i + 1] = s.femur;
        buf_[3 * i + 2] = s.tibia;
    }
    driver_.setAngles(buf_);
}

void LegController::setJointTargets(const float dt_s, const Array<float, NUM_JOINTS>& q_target) {
    if (faulted_) return;

    const float max_step = MAX_SLEW_RAD_S * dt_s;
    for (int i = 0; i < NUM_JOINTS; i++) {
        float step = q_target[i] - q_[i];
        step = fmaxf(-max_step, fminf(max_step, step));
        q_[i] += step;
    }
    writeServos();
}

bool LegController::moveToDefault(const float dt_s, const float speed_rad_s) {
    if (faulted_) return false;

    const float max_step = speed_rad_s * dt_s;
    bool done = true;
    for (int i = 0; i < NUM_JOINTS; i++) {
        const float e = q_default_[i] - q_[i];
        if (fabsf(e) > 1e-3f) done = false;
        q_[i] += fmaxf(-max_step, fminf(max_step, e));
    }
    writeServos();
    return done;
}

void LegController::estop() {
    faulted_ = true;
    // TODO: if ServoDriver can disable torque / go limp, call it here.
}