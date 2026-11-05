#ifndef IK_h
#define IK_h

#include "types.h"

struct IKConfig {
    float body_coxa_x_mm, femur_length_mm, tibia_length_mm, c_min, c_max;
};


bool IK(const IKConfig& cfg, Vec3 target_foot_pos_lf, Theta3& out_angles_deg);

Theta3 invertAngles(const Theta3& angles_deg);

#endif