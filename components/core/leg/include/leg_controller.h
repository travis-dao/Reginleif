#ifndef leg_controller_h
#define leg_controller_h

#include "leg.h"
#include "pca9685.h"

namespace Movement {
    constexpr float MAX_STEP_LENGTH_MM = 0.0f;
    constexpr float STEP_HEIGHT_MM = 100.0f;
    constexpr float SWING_DURATION_S = 0.250f;

    constexpr float WALK_OPPOSITE_LEG_Z_OFFSET[4] = {
        5.0f, 5.0f, 10.0f, 10.0f
    };

    constexpr float WALK_ADJACENT_LEG_Z_OFFSET[4] = {
        -8.0f, -8.0f, -6.0f, -6.0f
    };
    
    constexpr int next_leg_gait[5] = {
        /* -1 -> */ 0,
        /*  0 -> */ 2,
        /*  1 -> */ 3,
        /*  2 -> */ 1,
        /*  3 -> */ 0,
    };
}

namespace Offset {
    constexpr Vec3 neutral_offset = { 150.0f, 150.0f, 0.0f };

    const Vec3 neutral_vector[4] = {
        {  1,  1, 1 },  // Front Left
        {  1, -1, 1 },  // Front Right
        { -1, -1, 1 },  // Back Right
        { -1,  1, 1 },  // Back Left
    };

    constexpr Vec3 body_offset = { 33.72f, 43.13f, 0.0f };
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

		int current_airborne_leg;
        bool grounded;
};

#endif