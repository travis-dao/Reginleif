#include "leg_controller.h"
#include "leg.h"
#include "leg_config.h"
#include "servo.h"
#include "frame.h"
#include "types.h"
#include <algorithm>
#include <cmath>

const float NUM_SERVOS = 12;

// --------------- init ---------------
LegController::LegController() {
    for (int i = 0; i < 4; i++) {
        legs_[i] = nullptr;
        nominal_leg_pos_bf_[i] = Offset::neutral_foot_pos_bf[i];
    }

    grounded_ = true;
    swing_leg_ = -1;
    last_swing_leg_ = -1;
    gap_remaining_s = 0.0f;
    state_ = MoveState::IDLE;
    gait_ = Gait::WALK;
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
void LegController::update(float dt_s, const ImuData& imu_data) {
    updateTwist(dt_s);

    // gait sequencing
    bool all_grounded = true;
    for (int i = 0; i < 4; i++) {
        if (!legs_[i]->grounded()) { 
            all_grounded = false;
            break;
        }
    }

    // swing just ended -> all-stance gap
    if (all_grounded && !grounded_) {
        swing_leg_ = -1;
        gap_remaining_s = MovementConfig::GAP_DURATION_S;
        gait_ = updateGaitOrder();
    }
    grounded_ = all_grounded;

    // start next swing??
    if (grounded_ && state_ != MoveState::IDLE) {
        gap_remaining_s -= dt_s;
        if (gap_remaining_s <= 0.0f) { // stance gap done -> start next swing
            if (state_ == MoveState::WALKING) {
                int next = MovementConfig::getNextLeg(gait_, swing_leg_);
                last_swing_leg_ = next;
                swing_leg_ = next;
                legs_[next]->startSwing(legs_[next]->nominalLandingPos());
            } else { // MoveState::STOPPING
                setState(MoveState::IDLE);
            }
        }
    }

    // leg movement calculations
    for (int i = 0; i < 4; i++) {
        legs_[i]->update(dt_s, target_angles_[i], imu_data);

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
    if (state_ == MoveState::IDLE) {
        twist_.curr.v = 0.0f;
        twist_.curr.w = 0.0f;
        twist_.geom = limit(twist_.cmd);
    } else if (state_ == MoveState::WALKING) {
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
        Vec3 step = Vec3{-(geom.v - geom.w * pos.y) * T, -(geom.w * pos.x) * T, 0.0f}; // total foot displacement during stance

        legs_[i]->setStance(vel, step);
    }
}

Gait LegController::updateGaitOrder() {
    if (fabsf(twist_.curr.v) < 5.0f && fabsf(twist_.curr.w) > 0.2)
        return twist_.curr.w > 0.0f ? Gait::TURN_L : Gait::TURN_R;
    else
        return Gait::WALK;
}


// --------------- commands ---------------
void LegController::startWalk() {
    swing_leg_ = -1; // for gap phase
    state_ = MoveState::WALKING;

    for (Leg* leg : legs_) leg->startStance(leg->currFootPos());
}
void LegController::requestStart() { 
    if (state_ == MoveState::IDLE || state_ == MoveState::STOPPING) setState(MoveState::WALKING);
}
void LegController::requestStop() { 
    if (state_ == MoveState::WALKING) setState(MoveState::STOPPING);
}


// --------------- setters ---------------
void LegController::setState(LegController::MoveState new_state) {
    LegController::MoveState old = state_;
    state_ = new_state;

    if (old == MoveState::IDLE && new_state == MoveState::WALKING) {
        for (Leg* leg : legs_) leg->resumeStance();
        gap_remaining_s = 0.0f;
    } else if (new_state == MoveState::IDLE) { // MoveState::STOPPING -> MoveState::IDLE
        swing_leg_ = -1;
        for (Leg* leg : legs_) leg->hold();
    }
}
void LegController::setBodyCommand(const float v_mm_s, const float w_rad_s) { twist_.cmd = {v_mm_s, v_mm_s}; }


// --------------- getters ---------------
bool LegController::grounded() const { return grounded_; }
bool LegController::idle() const { return state_ == MoveState::IDLE; }
int LegController::swingLeg() const { return swing_leg_; }
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

Array<Vec3, 4> LegController::landingTargetsBody() const {
	Array<Vec3, 4> out;
	for (int i = 0; i < 4; i++) out[i] = legToBodyFrame(i, legs_[i]->nominalLandingPos());
	return out;
}

// [(leg, t_start, t_end)] of predicted swings, seconds from now, including gaps
SwingSchedule LegController::swingSchedule(float horizon_s) const {
    if (state_ == MoveState::IDLE) return {}; // MoveState::IDLE = no predicted swing

    const float T = MovementConfig::SWING_DURATION_S;
    const float G = MovementConfig::GAP_DURATION_S;

    SwingSchedule out;

    float t_next;

    // handle swing currently in progress
    if (swing_leg_ >= 0) {
        const float t_end = swingTimeRemainingS();

        out.emplace_back(swing_leg_, 0.0f, t_end);
        t_next = t_end + G;
    } else { // gap w/ no active swing
        t_next = fmaxf(gap_remaining_s, 0.0f);
    }

    // MoveState::STOPPING = finish current swing, no more afterwards
    if (state_ == MoveState::STOPPING) return out;

    int last = last_swing_leg_;

    // predict future swings
    while (t_next < horizon_s) {
        last = MovementConfig::getNextLeg(gait_, last_swing_leg_);

        out.emplace_back(last, t_next, t_next + T);

        t_next += T + G;
    }

    return out;
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