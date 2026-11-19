#ifndef IMU_SAMPLE_h
#define IMU_SAMPLE_h

#include <cmath>
#include <cstdint>
#include "types.h"

// One IMU reading, in the units and frame the policy expects.
// Body frame: x forward, y left, z up. Angles in radians, gyro in rad/s.
struct ImuSample {
    float roll  = 0.0f;                 // rad, ZYX convention
    float pitch = 0.0f;                 // rad
    float yaw   = 0.0f;                 // rad, logging only (policy never uses heading)
    Vec3  gyro_body = Vec3::Zero();     // rad/s
    uint32_t stamp_us = 0;              // esp_timer time of the reading (staleness checks)
    bool  valid = false;                // false on read error / non-finite data

    // Unit gravity direction in the body frame: R^T * [0, 0, -1]. z = -1 when upright.
    Vec3 gravityBody() const {
        const float sr = sinf(roll),  cr = cosf(roll);
        const float sp = sinf(pitch), cp = cosf(pitch);
        return Vec3(sp, -cp * sr, -cp * cr);
    }
};

#endif