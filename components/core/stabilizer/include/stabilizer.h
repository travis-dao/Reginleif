#ifndef STABILIZER_h
#define STABILIZER_h

#include "esp_err.h"
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
        esp_err_t update(const float dt_s, SharedData& out);

        Vec3 getAngles() const { return Vec3 { angles_deg.roll, -angles_deg.pitch, 0.0f }; }
        Vec3 getGyro()   const { return gyro_dps; }

    private:
        void updatePID( float dt_s, const Vec3& angles_in_deg, const Vec3& gyro_dps);
        esp_err_t updateImu();
        Mat3 getRotationMatrix(const Vec3& a);

		Ism330 imu_;
		IMU::EulerAngles angles_deg;
        Vec3 target_angles_deg;
        Vec3 gyro_dps;
        Mat3 rot_mat;

        float K_p;
        float K_i;
        float K_d;

        Vec3 integral_sum;
};

#endif