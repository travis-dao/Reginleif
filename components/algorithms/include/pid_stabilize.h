#ifndef PID_STABILIZE_h
#define PID_STABILIZE_h

#include "types.h"
#include "ism330.h"
#include "rtos_utils.h"

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
        void updatePID(const float dt_s, const Vec3& angles_in, const Vec3& gyro_dps, const Feedforward& ff);
        esp_err_t update(const float dt_s, SharedData& out_data, const Feedforward& ff);

        Mat3 rot_mat;
        Vec3 target_angles = Vec3 { 0.0f, 0.0f, 0.0f };

        Vec3 getAngles() const { return Vec3 { angles.roll, -angles.pitch, 0.0f }; }
        Vec3 getGyro()   const { return gyro_dps_; }

    private:
		Ism330 imu_;
		IMU::EulerAngles angles;
        Vec3 gyro_dps_ = Vec3 { 0.0f, 0.0f, 0.0f };

        float K_p;
        float K_i;
        float K_d;

        Vec3 integral_sum;

        Mat3 getRotationMatrix(const Vec3& compensation_angles);
};

#endif