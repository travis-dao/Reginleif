#include "motion_subsystem.h"

MotionSubsystem::MotionSubsystem(const Config& cfg)
    : mpc_(cfg.mpc_cfg),
      leg_controller_(cfg.leg_controller_),
      state_estimator_(cfg.state_estimator_cfg),
      cfg_(cfg)
{}

void MotionSubsystem::update(const float dt_s, const ImuData& imu) {
    // mpc cooking
    step(dt_s, imu);

    leg_controller_.update(dt_s, imu);
}

const MPCResult* MotionSubsystem::step(const float dt_s, const ImuData& imu) {
    time_since_ += dt_s;
    time_engaged_ += dt_s;

    if (time_since_ < cfg_.solve_period_s) {
        return lastResult_ ? &*lastResult_ : nullptr;
    }

    time_since_ = 0.0f;

    const State s0 = state_estimator_.state();

    if (!cfg_.yaw_des) {
        cfg_.yaw_des = s0.yaw;
    }

    auto x_ref = reference(s0);
    auto contact = contactSchedule();
    auto feet = footRelPos(s0, x_ref, contact);

    MPCProblem prob = {
        .x0 = s0,
        .x_ref = x_ref,
        .foot_rel_pos = feet,
        .contact = contact
    };

    lastResult_ = mpc_.solve(prob);
    const MPCResult& res = *lastResult_;

    if (res.success) { // on failure keep the last good offsets
        const Leg3& forces  = res.u0.foot_ground_force;   // Input::foot_ground_force, world frame
        
        // Convert forces to foot position corrections
        offsets_ = toOffsets(forces, contact[0], s0.yaw);
    }

    leg_controller_.applyMpcCorrections(offsets_);
    return &res;
}

/// Constant-velocity reference at steps 1..N: level body, fixed height, forward speed.
Vector<StateVec> MotionSubsystem::reference(const State& s0) const {
    const float dt = mpc_.config().dt;
    const float yaw = *cfg_.yaw_des;
    const Vec3f v_w = rotationMatrixYaw(yaw) * Vec3f(cfg_.vx_des_mps, 0.0f, 0.0f);

    Vector<StateVec> out;
    out.reserve(mpc_.N());

    for (int i = 0; i < mpc_.N(); i++) {
        VecXf x = VecXf::Zero(NX);
        x[2] = yaw;
        Vec3f pos = s0.pos + v_w * (i + 1) * dt;
        pos.z() = cfg_.z_des_m;
        x.segment<3>(3) = pos;
        x.segment<3>(9) = v_w;
        x[12] = s0.g; // check gravity slow = State.g
        out.push_back(std::move(x));
    }

    return out;
}

// Predicted stance/swing per step -> gait table
Contact MotionSubsystem::contactSchedule() const {
    const int N = mpc_.N();
    const float dt = mpc_.config().dt;

    Contact contact(N);
    for (auto& c : contact) {
        c.fill(true);
    }
    
    for (const auto& [leg, t0, t1] : leg_controller_.swingSchedule(N * dt)) {
        for (int i = 0; i < N; i++) {
            const float tm = (i + 0.5f) * dt;

            if (t0 <= tm && tm <= t1) {
                contact[i][leg] = false;
            }
        }
    }

    return contact;
}

/// Foot minus CoM, world frame (m), per step.
/// Planted feet are fixed in the WORLD (so r shrinks/grows as the CoM advances);
/// a foot that lands inside the horizon is planted at its nominal touchdown point.
Vector<FootRel> MotionSubsystem::footRelPos(const State& s0, Vector<StateVec>& x_ref, const Contact contact) {
    const Mat3f R0 = rotationMatrixYaw(s0.yaw);
    const auto feet_body = leg_controller_.footPositionsBody();
    const auto land_body = leg_controller_.landingTargetsBody();

    FootRel world, land_bf;
    Array<bool, NUM_LEGS> airborne;
    for(int i = 0; i < NUM_LEGS; i++) {
        world[i] = s0.pos + R0 * mmToM(feet_body[i]);
        land_bf[i] = mmToM(land_body[i]);
        airborne[i] = !contact[0][i];
    }

    Vector<FootRel> out;
    for (int i = 0; i < mpc_.N(); i++) {
        // COM @ start of step i
        const VecXf com = (i == 0) ? s0.pos : Vec3f(x_ref[i - 1].segment<3>(3));
        const Mat3f R = rotationMatrixYaw(x_ref[i][2]);

        FootRel r{};
        for (int j = 0; j < NUM_LEGS; j++) {
            if (contact[i][j]) {
                if (airborne[j]) { // touch down
                    world[j] = com + R * land_bf[i];
                    airborne[j] = false;
                }
            } else {
                airborne[j] = true;
            }
            r[i] = world[i] - com;
        }
        out.push_back(r);
    }

    return out;
}

/// MPC forces -> leg-frame foot offsets (mm).
/// The kinematic standing pose already carries the static load, so only the deviation from an even weight share is turned into an offset.
Array<Vec3, NUM_LEGS> MotionSubsystem::toOffsets(const FootRel& forces, const Array<bool, NUM_LEGS>& contact0, float yaw) const {
    int n_st = 0;
    for (bool c : contact0) {
        n_st += c ? 1 : 0;
    }
    n_st = std::max(n_st, 1);
    const float share = mpc_.config().mass * abs(GRAVITY) / n_st;

    Array<Vec3, NUM_LEGS> fw;
    for (int i = 0; i < NUM_LEGS; i++) {
        Vec3f f = Vec3f::Zero();
        if (contact0[i]) {
            f = forces[i];
            f.z() -= share;
        }
        fw[i] = Vec3(f.x(), f.y(), f.z());
    }

    const float ramp = (cfg_.ramp_s > 0.0f) ? fmin(1.0f, time_engaged_ / cfg_.ramp_s) : 1.0f;
    const auto corr_w = footForcesToPositions(fw, cfg_.gain_mm_per_n * ramp, cfg_.max_correction_mm);

    Array<Vec3, NUM_LEGS> out;
    for (int i = 0; i < NUM_LEGS; i++) {
        out[i] = worldVectorToLegFrame(corr_w[i], yaw);
    }

    return out;
}

/// FL, FR, BR, BL foot positions in world coordinates (m).
FootRel MotionSubsystem::footPositionsWorld(const State& s0) const {
    const Mat3f R = rotationMatrixYaw(s0.yaw);
    const auto feet_body = leg_controller_.footPositionsBody();

    FootRel out;
    for (int i = 0; i < NUM_LEGS; i++) {
        out[i] = s0.pos + R * mmToM(feet_body[i]);
    }

    return out;
}