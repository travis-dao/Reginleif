#include "leg_controller.h"
#include "servo.h"

const float k_num_servos = 12;

// Leg-frame foot position -> true body-frame position
Vec3 legToBodyFrame(int leg, const Vec3& foot_pos_lf) {
	return Offset::leg_base_pos_bf[leg] + foot_pos_lf;
}

Vec3 bodyToLegFrame(int leg, const Vec3& foot_pos_bf) {
	return foot_pos_bf - Offset::leg_base_pos_bf[leg];
}

LegController::LegController()
    : fp_()
{
    for (int i = 0; i < 4; i++) {
        legs_[i] = nullptr;
    }

    grounded = true;
    this->swing_leg = -1;
}

void LegController::init(i2c_master_bus_handle_t bus) {
    // Initialize servo driver and connect to master I2C bus
    Pca9685::Config cfg{};
    cfg.bus = bus;
    cfg.pwm_freq_hz = ServoConfig::SERVO_FREQ;
    ESP_ERROR_CHECK(this->servo_driver_.init(cfg));

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
}

void LegController::update(float dt_s, const ImuData& imu_data) {
    // gait sequencing
    bool all_grounded = true;
    for (int i = 0; i < 4; i++) {
        if (!legs_[i]->isGrounded()) { 
            all_grounded = false;
            break;
        }
    }
    this->grounded = all_grounded;

    // only start next swing sequence if all legs are grounded
    if (this->grounded) {
        // get next leg
        int next = Movement::next_leg_gait[this->swing_leg + 1];
        this->swing_leg = next;

        Vec3 nominal_landing_pos_bf = legToBodyFrame(next, this->legs_[next]->getNominalLandingPos());

        // run foot plan MPC
        const auto result = this->fp_.planFoothold({
            .current_foot_positions_bf = footPositionsBody(),
            .swing_leg = next,
            .nominal_foothold_bf = nominal_landing_pos_bf
        });

        // valid result?
        const Vec3 landing_pos_bf = result.valid ? result.foothold_bf : nominal_landing_pos_bf;

        // start swing sequence of target leg
        this->legs_[next]->startSwing(bodyToLegFrame(next, landing_pos_bf));
    }

    // leg movement calculations
    for (int i = 0; i < 4; i++) {
        this->legs_[i]->update(dt_s, target_angles[i], imu_data);

        // update servos (burst write)
        this->target_angles_buffer[i * 3 + 0] = this->target_angles[i].coxa;
        this->target_angles_buffer[i * 3 + 1] = this->target_angles[i].femur;
        this->target_angles_buffer[i * 3 + 2] = this->target_angles[i].tibia;
    }
    servoSetAngles(this->servo_driver_, this->target_angles_buffer, k_num_servos);
}

// current foot pos, body frame
Array<Vec3, 4> LegController::footPositionsBody() const {
	Array<Vec3, 4> out;
	for (int i = 0; i < 4; ++i) {
        out[i] = legToBodyFrame(i, this->legs_[i]->getCurrentLegPos());
    }
	return out;
}

// Where each foot touches down. Only the airborne leg differs from its current position. The next
// leg to lift lands after the MPC horizon ends (horizon = one swing), so it doesn't need one.
Array<Vec3, 4> LegController::landingTargetsBody() const {
	Array<Vec3, 4> out;
	for (int i = 0; i < 4; ++i) {
		const Vec3 foot_pos_lf = 
            (i == this->swing_leg) 
            ? this->legs_[i]->getNominalLandingPos() 
            : this->legs_[i]->getCurrentLegPos();
		out[i] = legToBodyFrame(i, foot_pos_lf);
	}
	return out;
}