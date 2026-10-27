#ifndef leg_controller_h
#define leg_controller_h

#include "leg.h"
#include "pca9685.h"

namespace Movement {
    constexpr float MAX_STEP_LENGTH_MM = 0.0f;
    constexpr float STEP_HEIGHT_MM = 100.0f;
    constexpr float SWING_DURATION_S = 0.250f;
    
    constexpr int next_leg_gait[5] = {
        /* -1 -> */ 0,
        /*  0 -> */ 2,
        /*  1 -> */ 3,
        /*  2 -> */ 1,
        /*  3 -> */ 0,
    };
}

namespace Offset {
    // Unsigned magnitudes
    constexpr Vec3 foot_offset_mag     = { 150.0f, 150.0f, 0.0f };  // foot relative to its leg base
    constexpr Vec3 leg_base_offset_mag = { 33.72f, 43.13f, 0.0f };  // leg base relative to body center

    // Per-leg sign for each axis (FL, FR, BR, BL)
    const Vec3 quadrant_sign[4] = {
        { +1, +1, 1 },  // Front Left
        { +1, -1, 1 },  // Front Right
        { -1, -1, 1 },  // Back Right
        { -1, +1, 1 },  // Back Left
    };

    // Leg base position, body frame
    const Vec3 leg_base_pos[4] = {
        quadrant_sign[0] * leg_base_offset_mag,
        quadrant_sign[1] * leg_base_offset_mag,
        quadrant_sign[2] * leg_base_offset_mag,
        quadrant_sign[3] * leg_base_offset_mag,
    };

    // Neutral foot position, leg frame
    const Vec3 foot_neutral_leg[4] = {
        quadrant_sign[0] * foot_offset_mag,
        quadrant_sign[1] * foot_offset_mag,
        quadrant_sign[2] * foot_offset_mag,
        quadrant_sign[3] * foot_offset_mag,
    };

    // Neutral foot position, body frame
    const Vec3 foot_neutral_body[4] = {
        leg_base_pos[0] + foot_neutral_leg[0],
        leg_base_pos[1] + foot_neutral_leg[1],
        leg_base_pos[2] + foot_neutral_leg[2],
        leg_base_pos[3] + foot_neutral_leg[3],
    };
}

class LegController {
    public:
        LegController();

        void update(const float dt_s, const SharedData& shared_data);
        esp_err_t init(i2c_master_bus_handle_t bus);
        
        bool isGrounded() { return this->grounded; }
		int getCurrentAirborneLeg() { return this->current_airborne_leg; }

    private:
        Leg* legs_[4];
		Pca9685 servo_driver_;

        Theta3* target_angles[4];
        float target_angles_buffer[12];

		int current_airborne_leg;
        bool grounded;
};

#endif