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
        void init(i2c_master_bus_handle_t bus);
        void update(const float dt_s, ImuData& out);

        Vec3 angles() const { return Vec3 { angles_deg_.roll, -angles_deg_.pitch, 0.0f }; }
        Vec3 gyro()   const { return gyro_dps_; }

    private:
        void updatePID( float dt_s, const Vec3& angles_in_deg, const Vec3& gyro_dps);
        void updateImu();
        Mat3 getRotationMatrix(const Vec3& a);

		Ism330 imu_;
		IMU::EulerAngles angles_deg_;
        Vec3 target_angles_deg_;
        Vec3 gyro_dps_;
        Mat3 rot_mat_;

        float K_p_;
        float K_i_;
        float K_d_;

        Vec3 integral_sum_;
};

#endif