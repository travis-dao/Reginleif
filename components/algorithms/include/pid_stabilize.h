#ifndef PID_STABILIZE_h
#define PID_STABILIZE_h

#include "types.h"
#include "config.h"

class Stabilizer {
    public:
        struct Config {
            float K_p;
            float K_i;
            float K_d;
        };
        
        Stabilizer(const Config& config);
        void update(float dt_s, const Vec3& current_angles);

        Mat3 rot_mat;
        Vec3 target_angles = Vec3 { 0.0f, 0.0f, 0.0f };

    private:
        float K_p;
        float K_i;
        float K_d;

        Vec3 integral_sum;
        Vec3 last_error;

        Mat3 getRotationMatrix(const Vec3& compensation_angles);
};

#endif