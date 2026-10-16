#include <math.h>

#include "pid_stabilize.h"
#include "config.h"

Stabilizer::Stabilizer(const Config& config) {
    K_p = config.K_p;
    K_i = config.K_i;
    K_d = config.K_d;

    this->integral_sum = Vec3 { 0.0f, 0.0f, 0.0f };
    this->last_error = Vec3 { 0.0f, 0.0f, 0.0f };

    rot_mat = Mat3 {{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    }};
}

void Stabilizer::update(float dt_s, const Vec3& current_angles) {

    // error
    Vec3 error = target_angles - current_angles;

    // pid stuff
    Vec3 proportional_term = error * Configs::PID::K_p;

    this->integral_sum = this->integral_sum + (error * dt_s);

    Vec3 integral_term = this->integral_sum * Configs::PID::K_i;

    Vec3 derivative_term = (error - this->last_error) / dt_s * Configs::PID::K_d;

    // add the P, I, D terms
    Vec3 compensation_angles_deg = proportional_term + integral_term + derivative_term;

    Vec3 compensation_angles = compensation_angles_deg * (M_PI / 180);

    this->last_error = error;

    this->rot_mat = getRotationMatrix(compensation_angles);
}

Mat3 Stabilizer::getRotationMatrix(const Vec3& compensation_angles) {
    Mat3 rotation_matrix_x {{
        {1, 								0, 							0},
        {0, cos(compensation_angles.x), -sin(compensation_angles.x)},
        {0, sin(compensation_angles.x), cos(compensation_angles.x)}
    }};
    Mat3 rotation_matrix_y {{
        {cos(compensation_angles.y),  0, sin(compensation_angles.y)},
        {0, 								1, 							 0},
        {-sin(compensation_angles.y), 0, cos(compensation_angles.y)}
    }};
    Mat3 rotation_matrix_z {{
        {cos(compensation_angles.z), -sin(compensation_angles.z), 0},
        {sin(compensation_angles.z), cos(compensation_angles.z),  0},
        {0, 							   0, 							 1}
    }};

    Mat3 rotation_matrix = rotation_matrix_x * rotation_matrix_y * rotation_matrix_z;

    return rotation_matrix;
}