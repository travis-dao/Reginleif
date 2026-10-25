#include "leg_controller.h"
#include "servo.h"
#include <algorithm>

static const float kTiltAtt[4][2] = {   // {roll, pitch} deg, TODO - logged with stabilizer off
    { 0.0f, 0.0f },   // leg 0 airborne
    { 0.0f, 0.0f },   // leg 1
    { 0.0f, 0.0f },   // leg 2
    { 0.0f, 0.0f },   // leg 3
};

static float z_off_[4]      = {0, 0, 0, 0};
static float z_off_goal_[4] = {0, 0, 0, 0};
static const float TILT_MM = 6.0f;          // start small, then raise to what worked
static const float MAX_TILT_MM_PER_S = 100.0f;
static const float AT_GOAL_MM = 1.0f;

LegController::LegController() {
    for (int i = 0; i < 4; i++) {
        legs_[i] = nullptr;
    }

    grounded = true;
    this->current_airborne_leg = -1;
}

esp_err_t LegController::init(i2c_master_bus_handle_t bus) {
    // Initialize servo driver and connect to master I2C bus
    Pca9685::Config cfg{};
    cfg.bus = bus;
    cfg.pwm_freq_hz = ServoConfig::SERVO_FREQ;
    esp_err_t e = this->servo_driver_.init(cfg);

    if (e != ESP_OK) {
        printf("Failed to initialize PCA9685: %s\n", esp_err_to_name(e));
        return e;
    }

    printf("Initialized Servo Driver\n");
    vTaskDelay(pdMS_TO_TICKS(1000));


    // Instantiate legs
    for (int i = 0; i < 4; i++) {
        this->legs_[i] = new Leg({
            i,
            this->servo_driver_,
        });
    }

    printf("Initialized LegController\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    return ESP_OK;
}

static float approach(float cur, float goal, float max_step) {
    return cur + std::clamp(goal - cur, -max_step, max_step);
}

void LegController::update(float dt_s, const SharedData& shared_data, Feedforward& ff) {
    // ---- 1. gait sequencing
    bool all_grounded = true;
    for (int i = 0; i < 4; i++) {
        if (!legs_[i]->isGrounded()) { 
            all_grounded = false;
            break;
        }
    }
    this->grounded = all_grounded;

    if (this->grounded) {
        int next = Movement::next_leg_gait[this->current_airborne_leg + 1];
        this->current_airborne_leg = next;
        legs_[next]->setState(SWING);
    }

    int lifting = this->current_airborne_leg;
    if (lifting >= 0.0f && legs_[lifting]->getSwingPhase() > 0.7f) {
        lifting = Movement::next_leg_gait[lifting + 1];
    }
    setTiltGoal(lifting);

    const float step = MAX_TILT_MM_PER_S * dt_s;
    bool tilt_moving = false;
    float remaining = 0.0f;

    for (int i = 0; i < 4; i++) {
        z_off_[i] = approach(z_off_[i], z_off_goal_[i], step);
        float gap = fabsf(z_off_goal_[i] - z_off_[i]);
        remaining = std::max(remaining, gap);
        if (gap > AT_GOAL_MM) tilt_moving = true;
    }

    // expected attitude, ramped with the tilt progress (0 = just switched, 1 = settled)
    float p = 1.0f - std::clamp(remaining / (2.0f * TILT_MM), 0.0f, 1.0f);
    if (lifting >= 0) {
        ff.attitude_deg = Vec3{ p * kTiltAtt[lifting][0], p * kTiltAtt[lifting][1], 0.0f };
    } else {
        ff.attitude_deg = Vec3{0, 0, 0};
    }
    ff.rate_dps        = Vec3{0, 0, 0}; // TODO tune
    ff.freeze_integral = tilt_moving;


    for (int i = 0; i < 4; i++) {
        legs_[i]->update(dt_s, shared_data, z_off_[i], *target_angles[i]);
    }


    // burst write
    for (int i = 0; i < 4; ++i) {
        target_angles_buffer[i * 3 + 0] = target_angles[i]->coxa;
        target_angles_buffer[i * 3 + 1] = target_angles[i]->femur;
        target_angles_buffer[i * 3 + 2] = target_angles[i]->tibia;
    }
    servoSetAngles(servo_driver_, target_angles_buffer, 12);
}

void LegController::setTiltGoal(int current_airborne_leg) {
    for (int i = 0; i < 4; i++) {
        int s = Offset::quadrant_sign[i].x * Offset::quadrant_sign[current_airborne_leg].x
              + Offset::quadrant_sign[i].y * Offset::quadrant_sign[current_airborne_leg].y;   // 2 = self, 0 = adjacent, -2 = diagonal
        if      (s == 0)  z_off_goal_[i] = -TILT_MM;     // adjacent: push down
        else if (s == -2) z_off_goal_[i] = +TILT_MM;     // diagonal: pull up
        else              z_off_goal_[i] = 0.0f;         // the lifting leg itself
    }
}