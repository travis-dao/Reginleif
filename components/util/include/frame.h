#ifndef MOTION_UTIL_h
#define MOTION_UTIL_h

#include <array>

#include "eigen.h"
#include "types.h"
#include "leg.h"
 
// Map world-frame ground reaction forces (N) to heuristic foot corrections (mm, world frame)
Array<Vec3, 4> footForcesToPositions(const Array<Vec3, 4>& foot_forces, float gain_mm_per_n = 0.5f, float max_correction_mm = 30.0f);
 
Mat3f rotationMatrixYaw(float yaw);
 
Vec3 legToBodyFrame(int leg_idx, const Vec3& foot_pos_lf);
Vec3 bodyToLegFrame(int leg_idx, const Vec3& foot_pos_bf);
Vec3 legToWorldFrame(int leg_idx, const Vec3& foot_pos_lf, const Vec3& body_pos_wf, float yaw);
 
// Rotate a displacement vector from world axes into body-aligned leg axes (yaw only, no translation)
Vec3 worldVectorToLegFrame(const Vec3& vector_wf, float yaw);


const float MM_TO_M = 1e-3f;
Vec3f mmToM(const Vec3& v);

Vec3f toEigen(const Vec3& v);
Vec3 toVec3(const Vec3f& v);

#endif