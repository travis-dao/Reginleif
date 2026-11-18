#include "leg_controller.h"
#include "eigen.h"
#include "leg.h"
#include "leg_config.h"
#include "servo.h"
#include "frame.h"
#include "types.h"
#include <algorithm>
#include <optional>

const float NUM_SERVOS = 12;

// --------------- helpers ---------------
Mat3 rot_rp(const float roll, const float pitch) {
    const float cr = cosf(roll);
    const float sr = sinf(roll);
    const float cp = cosf(pitch);
    const float sp = sinf(pitch);

    return Mat3{{{
        {{cp,   sp * sr, sp * cr}},
        {{0.0f, cr,      -sr}},
        {{-sp,  cp * sr, cp * cr}}
    }}};
}

// --------------- init ---------------
LegController::LegController() {
    for (int i = 0; i < 4; i++) {
        legs_[i] = nullptr;
        nominal_leg_pos_bf_[i] = Offset::neutral_foot_pos_bf[i];
    }

    grounded_ = true;
    swing_leg_ = -1;
    last_swing_leg_ = -1;
    move_state_ = MoveState::IDLE;
    gait_ = Gait::WALK_F;
    vel_cmd_xy_ = Vec3{0.0f, 0.0f, 0.0f};
    body_roll_ = body_pitch_ = 0.0f;

    const float T = MovementConfig::SWING_DURATION_S;
    const float G = MovementConfig::GAP_DURATION_S;

    gait_clock_.duty_zheta = 4.0f; // 2.7~3.0 for faster gaits
    gait_clock_.duty = (1.0f - T) / (gait_clock_.duty_zheta * (T + G));
    gait_clock_.period_s = T / (1.0f - gait_clock_.duty_zheta);
    gait_clock_.psi_ = {0.0f, 0.0f, 0.0f, 0.0f,};
    gait_clock_.grounded_s = 0.0f;
}

void LegController::init(i2c_master_bus_handle_t bus) {
    // Initialize servo driver and connect to master I2C bus
    Pca9685::Config cfg{};
    cfg.bus = bus;
    cfg.pwm_freq_hz = ServoConfig::SERVO_FREQ;
    ESP_ERROR_CHECK(servo_driver_.init(cfg));

    printf("Initialized Servo Driver\n");
    vTaskDelay(pdMS_TO_TICKS(1000));


    // Instantiate legs
    for (int i = 0; i < 4; i++) legs_[i] = new Leg({i});

    printf("Initialized LegController\n");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// --------------- update ---------------
void LegController::update(float dt_s, const State& state, const ImuCorrection& imu_correction) {
    updateTwist(dt_s);
    state_ = state;

    float yaw = state_.yaw;
    float vel_cmd = twist_.cmd.v;
    vel_cmd_xy_ = Vec3{vel_cmd * cosf(yaw), vel_cmd * sinf(yaw), 0.0f};

    // gait sequencing
    bool all_grounded = true;
    for (int i = 0; i < 4; i++) {
        if (!legs_[i]->grounded()) { 
            all_grounded = false;
            break;
        }
    }

    gait_clock_.grounded_s = all_grounded ? gait_clock_.grounded_s + dt_s : 0.0f;

    // swing just ended
    if (all_grounded && !grounded_) {
        swing_leg_ = -1;
        Gait new_gait = updateGaitOrder();
        if (new_gait != gait_) {
            gait_ = new_gait;
            if (last_swing_leg_ >= 0) {
                const float remaining =(1.0f - *std::min_element(gait_clock_.psi_.begin(), gait_clock_.psi_.end())) * gait_clock_.period_s;
                scheduleFrom(mostOverdueLeg(), remaining);
            }
        }
    }
    grounded_ = all_grounded;

    // start next swing??
    if (move_state_ == MoveState::WALKING) {
        advanceClock(dt_s);
    } else if (move_state_ == MoveState::STOPPING) {
        // clock frozen: no new swings, finish the one in flight, then idle after a gap
        if (grounded_ && gait_clock_.grounded_s >= MovementConfig::GAP_DURATION_S) {
            setState(MoveState::IDLE);
        }
    }

    // leg movement calculations
    for (int i = 0; i < 4; i++) {
        legs_[i]->update(dt_s, target_angles_[i], imu_correction);

        // update servos (burst write)
        target_angles_buffer_[i * 3 + 0] = target_angles_[i].coxa;
        target_angles_buffer_[i * 3 + 1] = target_angles_[i].femur;
        target_angles_buffer_[i * 3 + 2] = target_angles_[i].tibia;
    }
    servoSetAngles(servo_driver_, target_angles_buffer_, NUM_SERVOS);
}

void LegController::applyMpcCorrections(Array<Vec3, 4> offsets_lf) {
    for (int i = 0; i < 4; i++) legs_[i]->setMpcOffset(offsets_lf[i]);
}

void LegController::updateTwist(const float dt_s) {
    // update commanded motion
    if (move_state_ == MoveState::IDLE) {
        twist_.curr.v = 0.0f;
        twist_.curr.w = 0.0f;
        twist_.geom = limit(twist_.cmd);
    } else if (move_state_ == MoveState::WALKING) {
        Twist cmd = limit(twist_.cmd); // limited command
        twist_.geom = cmd; // update geo

        // current vels += clamped vel (accel * dt)
        twist_.curr.v += fmaxf(-limits_.a_v_mm_s2 * dt_s, fminf(limits_.a_v_mm_s2 * dt_s, cmd.v - twist_.curr.v));
        twist_.curr.w += fmaxf(-limits_.a_w_rad_s2 * dt_s, fminf(limits_.a_w_rad_s2 * dt_s, cmd.w - twist_.curr.w));
    }

    // calculate desired stance geometry
    Twist geom = twist_.geom;
    Twist curr = twist_.curr;
    float T = gait_cfg_.time_stance_s;
    for (int i = 0; i < 4; i++) {
        Vec3 pos = nominal_leg_pos_bf_[i];

        Vec3 vel = Vec3{-(curr.v - curr.w * pos.y), -(curr.w * pos.x), 0.0f}; // foot vel during stance
        Vec3 step = Vec3{(geom.v - geom.w * pos.y) * T, (geom.w * pos.x) * T, 0.0f}; // total foot displacement during stance

        legs_[i]->setStance(vel, step);
    }
}

Gait LegController::updateGaitOrder() {
    if (fabsf(twist_.curr.v) < 5.0f && fabsf(twist_.curr.w) > 0.2)
        return twist_.curr.w > 0.0f ? Gait::TURN_L : Gait::TURN_R;
    else
        return twist_.curr.v < 0 ? Gait::WALK_B : Gait::WALK_F;
}

void LegController::advanceClock(const float dt_s) {
    float dphase = dt_s / gait_clock_.period_s;
    for (int i = 0; i < 4; i++) {
        gait_clock_.psi_[i] += dphase;

        if (gait_clock_.psi_[i] >= 1.0f) {
            gait_clock_.psi_[i] -= 1.0;

            if (legs_[i]->grounded()) startSwing(i);
        }
    }
}


// --------------- commands ---------------
void LegController::startWalk() {
    swing_leg_ = -1; // for gap phase
    move_state_ = MoveState::WALKING;

    for (Leg* leg : legs_) leg->startStance(leg->currFootPos());
}


void LegController::startSwing(const int leg) {
    last_swing_leg_ = leg;
    swing_leg_ = leg;
    legs_[leg]->startSwing(nominalLandingPos(leg));
}

void LegController::requestStart() { 
    if (move_state_ == MoveState::IDLE || move_state_ == MoveState::STOPPING) setState(MoveState::WALKING);
}

void LegController::requestStop() { 
    if (move_state_ == MoveState::WALKING) setState(MoveState::STOPPING);
}


// --------------- setters ---------------
void LegController::setState(LegController::MoveState new_state) {
    LegController::MoveState old = move_state_;
    move_state_ = new_state;

    const float G = MovementConfig::GAP_DURATION_S;

    if (old == MoveState::IDLE && new_state == MoveState::WALKING) {
        for (Leg* leg : legs_) leg->resumeStance();
        Gait new_gait = updateGaitOrder();
        bool changed = new_gait != gait_;
        gait_ = new_gait;
        int first = changed && last_swing_leg_ >= 0 ? mostOverdueLeg() : MovementConfig::getNextLeg(gait_, last_swing_leg_);
        float push = (gait_ == Gait::WALK_F || gait_ == Gait::WALK_B) ? gait_cfg_.start_push_s : 0.0f;
        scheduleFrom(first, push);
    } else if (old == MoveState::STOPPING && new_state == MoveState::WALKING) {
        float delay;

        if (swing_leg_ >= 0) {
            delay = swingTimeRemainingS() + G;
        } else {
            delay = std::max(G - gait_clock_.grounded_s, 0.0f);
        }
        scheduleFrom(MovementConfig::getNextLeg(gait_, last_swing_leg_), delay);
    } else if (new_state == MoveState::IDLE) { // MoveState::STOPPING -> MoveState::IDLE
        swing_leg_ = -1;
        for (Leg* leg : legs_) leg->hold();
    }
}
void LegController::setBodyCommand(const float v_mm_s, const float w_rad_s) { twist_.cmd = {v_mm_s, v_mm_s}; }

void LegController::setBodyTilt(const float roll_rad, const float pitch_rad) {
    body_roll_ = roll_rad;
    body_pitch_ = pitch_rad;

    for (auto leg : legs_) {
        leg->setBodyTilt(rot_rp(roll_rad, pitch_rad).transpose());
    }
}


// --------------- getters ---------------
bool LegController::grounded() const { return grounded_; }

bool LegController::idle() const { return move_state_ == MoveState::IDLE; }

int LegController::swingLeg() const { return swing_leg_; }

int LegController::mostOverdueLeg() {
    int best = -1;
    int best_dist = -1.0;

    for (int i = 0; i < 4; i++) {
        Vec3 cur = legs_[i]->currFootPos();
        Vec3 target = nominalLandingPos(i); // change to 0?
        float dist = std::hypotf(target.x - cur.x, target.y - cur.y);
        if (dist > best_dist) {
            best = i;
            best_dist = dist;
        }
    }

    return best;
}

Vec3 LegController::nominalLandingPos(const int leg) {
    return legs_[leg]->nominalLandingPos(
        Vec3{state_.linear_vel[0] * M_TO_MM, state_.linear_vel[1] * M_TO_MM, 0.0f},
        vel_cmd_xy_,
        std::max(state_.pos[2] * M_TO_MM, 1.0f)
    );
}

GaitConfig LegController::gaitConfig() const { return gait_cfg_; }

float LegController::swingTimeRemainingS() const {
    if (swing_leg_ < 0) return 0.0f;
    else return legs_[swing_leg_]->swingTimeRemainingS();
}

Twist LegController::bodyTwist() const { return twist_.curr; }


// --------------- util ---------------
// current foot pos, body frame
Array<Vec3, 4> LegController::footPositionsBody() const {
	Array<Vec3, 4> out;
	for (int i = 0; i < 4; i++) out[i] = legToBodyFrame(i, legs_[i]->currFootPos());
	return out;
}

Array<Vec3, 4> LegController::landingTargetsBody() {
	Array<Vec3, 4> out;
	for (int i = 0; i < 4; i++) out[i] = legToBodyFrame(i, nominalLandingPos(i));
	return out;
}

// [(leg, t_start, t_end)] of predicted swings, seconds from now, including gaps
SwingSchedule LegController::swingSchedule(float horizon_s) const {
    if (move_state_ == MoveState::IDLE) return {}; // MoveState::IDLE = no predicted swing

    const float T = MovementConfig::SWING_DURATION_S;
    const float P = gait_clock_.period_s;

    SwingSchedule out;
    
    for (int i = 0; i < 4; ++i) {
        const Leg& leg = *legs_[i];

        if (!leg.grounded()) {
            out.emplace_back(i, 0.0f, leg.swingTimeRemainingS());
        }

        if (move_state_ == MoveState::STOPPING) {
            continue;
        }

        float t = (1.0f - gait_clock_.psi_[i]) * P;

        while (t < horizon_s) {
            out.emplace_back(i, t, t + T);
            t += P;
        }
    }

    std::sort(out.begin(), out.end(),
        [](const auto& a, const auto& b) {
            return std::get<1>(a) < std::get<1>(b);
        });

    return out;
}

void LegController::scheduleFrom(const int leg, const float delay_s) {
    // Make `first` swing in delay_s, then the rest in order, a quarter period apart
    Gait gait = gait_;
    auto order = MovementConfig::getOrder(gait);
    int i0 = order[0];
    float tau0 = delay_s / gait_clock_.period_s;
    for (int j = 0; j < 4; j++) {
        // psi may go negative -> "fires later than one period from now"
        gait_clock_.psi_[order[(i0 + j) % 4]] = 1.0f - (tau0 + j / 4.0f);
    }
}

Twist LegController::limit(Twist twist) const {
    float T = gait_cfg_.time_stance_s;
    float k = 1.0f; // fraction of original movement command we can keep

    // find most restrictive scale -> lowest
    for (Vec3 p : nominal_leg_pos_bf_) {
        float scale_x = abs(twist.v - twist.w * p.y) * T;
        float scale_y = abs(twist.w * p.x) * T;
        
        k = fminf(k, fminf(
            scale_x > 1e-9f ? MovementConfig::STEP_LENGTH_MM / scale_x : 1.0f,
            scale_y > 1e-9f ? gait_cfg_.max_step_y_mm / scale_y : 1.0f
        ));
    }

    return Twist { twist.v * k, twist.w * k }; // scale both v & w to preserve turning radius
}