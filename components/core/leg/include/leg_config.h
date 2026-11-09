#ifndef LEG_CONFIG_h
#define LEG_CONFIG_h

#include "types.h"

constexpr float FEMUR_LENGTH_MM = 122.0f;
constexpr float TIBIA_LENGTH_MM = 160.0f;
constexpr float BODY_COXA_X_MM = 15.0f;

const float c_max = FEMUR_LENGTH_MM + TIBIA_LENGTH_MM;
const float c_min = fabsf(FEMUR_LENGTH_MM - TIBIA_LENGTH_MM);

namespace Movement {
    constexpr float STEP_LENGTH_MM = 100.0f;
    constexpr float STEP_HEIGHT_MM = 60.0f;
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
    // in foot frame
    // (0, 0, 0) = hip of foot
    constexpr Vec3 foot_neutral_offset_mag = { 120.0f, 100.0f, -40.0f };
    // from body frame
    // (0, 0, 0) = center of body
    constexpr Vec3 leg_base_offset_mag = { 34.0f, 43.5f, 0.0f };

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

#endif