#ifndef LEG_CONFIG_h
#define LEG_CONFIG_h

#include <cmath>
#include "types.h"

// Geometry only. All gait config (MovementConfig, Gait) was removed.

constexpr float FEMUR_LENGTH_MM = 122.0f;
constexpr float TIBIA_LENGTH_MM = 160.0f;
constexpr float BODY_COXA_X_MM = 15.0f;

const float c_max = FEMUR_LENGTH_MM + TIBIA_LENGTH_MM;
const float c_min = fabsf(FEMUR_LENGTH_MM - TIBIA_LENGTH_MM);

namespace Offset {
    // in foot frame, (0, 0, 0) = hip of foot
    const Vec3 foot_neutral_offset_mag = Vec3(120.0f, 100.0f, -40.0f);
    // from body frame, (0, 0, 0) = center of body
    const Vec3 leg_base_offset_mag = { 34.0f, 43.5f, 0.0f };

    // Per-leg sign for each axis (FL, FR, BR, BL)
    const Array<Vec3, 4> quadrant_sign = {{
        { +1, +1, +1 },  // Front Left
        { +1, -1, +1 },  // Front Right
        { -1, -1, +1 },  // Back Right
        { -1, +1, +1 },  // Back Left
    }};

    // Neutral foot position, leg frame (used once, to derive the default joint pose)
    const Array<Vec3, 4> neutral_foot_pos_lf = {{
        quadrant_sign[0].cwiseProduct(foot_neutral_offset_mag),
        quadrant_sign[1].cwiseProduct(foot_neutral_offset_mag),
        quadrant_sign[2].cwiseProduct(foot_neutral_offset_mag),
        quadrant_sign[3].cwiseProduct(foot_neutral_offset_mag),
    }};
}

#endif