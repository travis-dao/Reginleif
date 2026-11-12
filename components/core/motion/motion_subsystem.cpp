#include "motion_subsystem.h"
#include "eigen.h"
#include "frame.h"
#include "motion_config.h"
#include "mpc.h"
#include "srb_model.h"
#include "state_estimator.h"
#include "types.h"
#include <cmath>

// --------------- helpers ---------------
float wrap(float a) {
    return fmodf(fmodf(a + PI, 2.0f * PI) + 2.0f * PI, 2.0f * PI) - PI;
}

// --------------- constructor ---------------
MotionSubsystem::MotionSubsystem(
    const Config& cfg,
    const MPC::Config& mpc_cfg, 
    const StateEstimator::Config& state_estimator_cfg, 
    LegController& leg_controller
) : 
    mpc_(mpc_cfg),
    leg_controller_(leg_controller),
    state_estimator_(state_estimator_cfg, leg_controller),
    cfg_(cfg)
{
    mode_ = STAND;
    cmd_.yaw_leash_rad = cfg.yaw_leash_rad;
    yaw_des_ = cfg.yaw_des.value_or(0.0f);
    state_ = state_estimator_.state();
    mpc_active_ = false;
}
// --------------- update ---------------
void MotionSubsystem::update(const float dt_s, const ImuData& imu) {
    auto state = estimate(dt_s, imu);

    // input
    // set vel and omega cmd -- TODO
    const float V = 0.8f * leg_controller_.gaitConfig().v_max_mm_s * MM_TO_M; // m/s
    const float R = 0.25f; // path radius, m
    if (true) { // fix - TODO
        cmd_.vel_cmd_mps = V;
        cmd_.yaw_rate_cmd = V / R;
    } else {
        cmd_.vel_cmd_mps = 0.0f;
        cmd_.yaw_rate_cmd = 0.0f;
    }
    
    // mpc
    if (!mpc_active_) {
        if (state.linear_vel.norm() <= SETTLE_VEL) {
            cfg_.z_des_m = state.pos[2];
            yaw_des_ = state.yaw;
            mpc_active_ = true;
            t_engaged_ = 0.0f;

            printf("MPC engaging, z_des(m)=%f", cfg_.z_des_m);
        }
    } else {
        auto res = step(dt_s, imu);

        if (res != NULL && !res->success) {
            printf("MPC solve failed, holding last offsets");
        }
    }

    // leg controller
    leg_controller_.update(dt_s, imu);
}

const MPCResult* MotionSubsystem::step(const float dt_s, const ImuData& imu) {
    t_since += dt_s;
    t_engaged_ += dt_s;

    if (t_since < cfg_.solve_period_s) {
        return lastResult_ ? &*lastResult_ : nullptr;
    }

    float elapsed = t_since;
    t_since = 0.0f;

    const State s0 = state_estimator_.state();
    leg_controller_.setBodyCommand(cmd_.vel_cmd_mps / MM_TO_M, cmd_.yaw_rate_cmd);
    updateMode(s0);
    
    yaw_des_ = s0.yaw + std::clamp(wrap(yaw_des_ - s0.yaw), -cfg_.yaw_leash_rad, cfg_.yaw_leash_rad);
    if (mode_ == WALK) {
        yaw_des_ += leg_controller_.bodyTwist().w * elapsed;
    }

    auto x_ref = reference(s0);
    auto contact = contactSchedule();
    auto feet = footRelWorld(s0, x_ref, contact);

    MPCProblem prob = {s0, x_ref, feet, contact};

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

void MotionSubsystem::updateMode(const State& s0) {
    bool moving = fabsf(cmd_.vel_cmd_mps) > 1e-3f || fabsf(cmd_.yaw_rate_cmd) > 1e-3f;

    if (moving) {
        leg_controller_.requestStart();
        mode_ = WALK;
    } else {
        leg_controller_.requestStop();
        if (mode_ == WALK and leg_controller_.idle()) {
            hold_xy_ = s0.toVector().head<2>();
            yaw_des_ = s0.yaw;
            mode_ = STAND;
        }
    }
}


// --------------- state estimator ---------------
void MotionSubsystem::reset(const Vec3f& p0, const Vec3f& v0, const ImuData& imu) { 
    state_estimator_.reset(p0, v0, imu);
}

State MotionSubsystem::estimate(const float dt_s, const ImuData& imu) {
    state_ = state_estimator_.update(dt_s, imu);
    return state_;
}

// --------------- other ---------------
// Constant-velocity reference at steps 1..N: level body, fixed height, forward speed.
Vector<StateVec> MotionSubsystem::reference(const State& s0) const {
    const float dt = mpc_.config().dt;

    float v = 0.0f;
    float w = 0.0f;

    if (mode_ == Mode::WALK) {
        const auto [v_mm, yaw_rate] = leg_controller_.bodyTwist();
        v = v_mm * MM_TO_M;
        w = yaw_rate;
    }

    Vec3f p = s0.pos;

    if (mode_ == STAND && hold_xy_.has_value()) {
        p.head<2>() = *hold_xy_;
    }

    Vector<StateVec> out;
    out.reserve(mpc_.N());

    for (int k = 0; k < mpc_.N(); ++k) {
        // midpoint integration of the arc
        const float yaw_mid = yaw_des_ + w * (k + 0.5f) * dt;
        const float yaw_end = yaw_des_ + w * (k + 1) * dt;

        p.head<2>() += rotationMatrixYaw(yaw_mid).topLeftCorner<2, 2>() * Eigen::Vector2f(v, 0.0f) * dt;

        p.z() = cfg_.z_des_m;

        VecXf x = VecXf::Zero(NX);

        x[IDX_THETA + 2] = yaw_end;
        x.segment<3>(IDX_POS) = p;
        x[IDX_OMEGA + 2] = w;  // yaw rate

        x.segment<3>(IDX_VEL) = rotationMatrixYaw(yaw_end) * Vec3f(v, 0.0f, 0.0f);

        x[IDX_G] = s0.g;

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

// Foot minus CoM, world frame (m), per step.
// Planted feet are fixed in the WORLD (so r shrinks/grows as the CoM advances);
// a foot that lands inside the horizon is planted at its nominal touchdown point.
Vector<FootRel> MotionSubsystem::footRelWorld(const State& s0, Vector<StateVec>& x_ref, const Contact contact) {
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
        const VecXf com = (i == 0) ? s0.pos : Vec3f(x_ref[i - 1].segment<3>(IDX_POS));
        const Mat3f R = rotationMatrixYaw(x_ref[i][IDX_THETA + 2]);

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

// relative foot positions, body frame, meters
Array<Vec3, NUM_LEGS> MotionSubsystem::footRelBody() {
    Array<Vec3, NUM_LEGS> out;
    auto pos_body = leg_controller_.footPositionsBody();

    for (int i = 0; i < NUM_LEGS; i++) {
        out[i] = pos_body[i] * MM_TO_M;
    }

    return out;
}

// MPC forces -> leg-frame foot offsets (mm).
// The kinematic standing pose already carries the static load, so only the deviation from an even weight share is turned into an offset.
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

    const float ramp = (cfg_.ramp_s > 0.0f) ? fminf(1.0f, t_engaged_ / cfg_.ramp_s) : 1.0f;
    const auto corr_w = footForcesToPositions(fw, cfg_.gain_mm_per_n * ramp, cfg_.max_correction_mm);

    Array<Vec3, NUM_LEGS> out;
    for (int i = 0; i < NUM_LEGS; i++) {
        out[i] = worldVectorToLegFrame(corr_w[i], yaw);
    }

    return out;
}