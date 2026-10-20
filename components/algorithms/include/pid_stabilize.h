#ifndef PID_STABILIZE_h
#define PID_STABILIZE_h

#include "types.h"
#include "mpu6050.h"
#include "ism330.h"

class Stabilizer {
    public:
        struct Config {
            float K_p;
            float K_i;
            float K_d;
        };

        Stabilizer(const Config& config);
        esp_err_t init(i2c_master_bus_handle_t bus);
        esp_err_t updateImu();
        void updatePID(const float dt_s, const Vec3& current_angles);
        esp_err_t update(const float dt_s, SharedData& out_data);

        Mat3 rot_mat;
        Vec3 target_angles = Vec3 { 0.0f, 0.0f, 0.0f };

        Vec3 getAngles() const { return Vec3 { angles.roll, -angles.pitch, 0.0f }; }

    private:
		Mpu6050 imu_;
		IMU::EulerAngles angles;

        float K_p;
        float K_i;
        float K_d;

        Vec3 integral_sum;
        Vec3 last_error;

        Mat3 getRotationMatrix(const Vec3& compensation_angles);
};

#endif