#include <cmath>

#include "frame.h"
#include "eigen.h"

Vec3f toEigen(const Vec3& v) { 
    return {v.x, v.y, v.z};
}
Vec3 toVec3(const Vec3f& v) { 
    return Vec3{v.x(), v.y(), v.z()};
}

Array<Vec3, 4> footForcesToPositions(const Array<Vec3, 4>& foot_forces, float gain_mm_per_n, float max_correction_mm) {
    Array<Vec3, 4> corrections;
    for (size_t i = 0; i < foot_forces.size(); i++) {
        const auto& f = foot_forces[i];

        // Position-controlled leg = stiff spring: to PUSH on the body with force f
        // the foot target must go the opposite way (deeper into the ground / back
        // for forward thrust). Hence the minus sign.
        float dx = -f.x * gain_mm_per_n;
        float dy = -f.y * gain_mm_per_n;
        float dz = -f.z * gain_mm_per_n;

        const float mag = sqrt(dx * dx + dy * dy + dz * dz);
        if (mag > max_correction_mm) {
            const float s = max_correction_mm / mag;
            dx *= s;
            dy *= s;
            dz *= s;
        }
        corrections[i] = Vec3{dx, dy, dz};
    }
    return corrections;
}

// ----------------------------------------------------------------------
Mat3f rotationMatrixYaw(const float yaw) {
    const float c = cosf(yaw), s = sinf(yaw);
    Mat3f R;
    R << c, -s, 0.0f,
         s,  c, 0.0f,
         0.0f, 0.0f, 1.0f;
    return R;
}

Mat3f rotationMatrixRPY(const float roll, const float pitch, const float yaw) {
    float cr = cosf(roll);  float sr = sinf(roll);
    float cp = cosf(pitch); float sp = sinf(pitch);
    float cy = cosf(yaw);   float sy = sinf(yaw);
    
    Mat3f Rz, Ry, Rx;

    Rz << cy, -sy, 0.0f,
          sy, cy, 0.0f,
          0.0f, 0.0f, 1.0f;
    Ry << cp, 0.0f, sp,
          0.0f, 1.0f , 0.0f,
          -sp, 0.0f, cp;
    Rx << 1.0f, 0.0f, 0.0f,
          0.0f, cr, -sr,
          0.0f, sr, cr;

    return Rz * Ry * Rx;
}

// ----------------------------------------------------------------------
Vec3 legToBodyFrame(int leg_idx, const Vec3& foot_pos_lf) {
    return Offset::leg_base_pos_bf[leg_idx] + foot_pos_lf;
}

Vec3 bodyToLegFrame(int leg_idx, const Vec3& foot_pos_bf) {
    return foot_pos_bf - Offset::leg_base_pos_bf[leg_idx];
}

Vec3 legToWorldFrame(int leg_idx, const Vec3& foot_pos_lf, const Vec3& body_pos_wf, float yaw) {
    const Vec3f p_b = toEigen(legToBodyFrame(leg_idx, foot_pos_lf));
    const Vec3f p_w = toEigen(body_pos_wf) + rotationMatrixYaw(yaw) * p_b;
    return toVec3(p_w);
}

Vec3 worldVectorToLegFrame(const Vec3& vector_wf, float yaw) {
    // Displacement vector: rotation only, no translation.
    return toVec3(rotationMatrixYaw(yaw).transpose() * toEigen(vector_wf));
}


Vec3f mmToM(const Vec3& v) {
    return Vec3f(v.x, v.y, v.z) * MM_TO_M;
}