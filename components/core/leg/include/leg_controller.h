#ifndef leg_controller_h
#define leg_controller_h

#include "leg.h"
#include "pca9685.h"
#include "mpc.h"

namespace Movement {
    constexpr float k_step_length_mm = 150.0f;
    constexpr float k_step_height_mm = 100.0f;
    constexpr float k_swing_duration_s = 0.250f;
    
    constexpr int next_leg_gait[5] = {
        /* -1 -> */ 0,
        /*  0 -> */ 2,
        /*  1 -> */ 3,
        /*  2 -> */ 1,
        /*  3 -> */ 0,
    };
}

namespace Offset {
    // in foot frame
    // (0, 0, 0) = hip of foot
    constexpr Vec3 foot_neutral_offset_mag = { 150.0f, 150.0f, 0.0f };
    // from body frame
    // (0, 0, 0) = center of body
    constexpr Vec3 leg_base_offset_mag = { 33.72f, 43.13f, -20.0f };

    // Per-leg sign for each axis (FL, FR, BR, BL)
    constexpr Array<Vec3, 4> quadrant_sign = {{
        { +1, +1, +1 },  // Front Left
        { +1, -1, +1 },  // Front Right
        { -1, -1, +1 },  // Back Right
        { -1, +1, +1 },  // Back Left
    }};

    // Leg base position, body frame
    const Array<Vec3, 4> leg_base_pos_bf = {{
        quadrant_sign[0] * leg_base_offset_mag,
        quadrant_sign[1] * leg_base_offset_mag,
        quadrant_sign[2] * leg_base_offset_mag,
        quadrant_sign[3] * leg_base_offset_mag,
    }};

    // Neutral foot position, leg frame
    const Array<Vec3, 4> neutral_foot_pos_lf = {{
        quadrant_sign[0] * foot_neutral_offset_mag,
        quadrant_sign[1] * foot_neutral_offset_mag,
        quadrant_sign[2] * foot_neutral_offset_mag,
        quadrant_sign[3] * foot_neutral_offset_mag,
    }};

    // Neutral foot position, body frame
    const Array<Vec3, 4> neutral_foot_pos_bf = {{
        leg_base_pos_bf[0] + neutral_foot_pos_lf[0],
        leg_base_pos_bf[1] + neutral_foot_pos_lf[1],
        leg_base_pos_bf[2] + neutral_foot_pos_lf[2],
        leg_base_pos_bf[3] + neutral_foot_pos_lf[3],
    }};
}

class LegController {
    public:
        LegController();

        void update(const float dt_s);
        esp_err_t init(i2c_master_bus_handle_t bus);
        
        bool isGrounded() { return this->grounded; }
		int getSwingLeg() { return this->swing_leg; }

        Array<Vec3, 4> footPositionsBody() const;
        Array<Vec3, 4> landingTargetsBody() const;

    private:
        Leg* legs_[4];
		Pca9685 servo_driver_;
        FootholdMpc mpc_;

        Array<Theta3, 4> target_angles;
        float target_angles_buffer[12];

		int swing_leg;
        bool grounded;
};

#endif