#ifndef IMU_WRAPPER_h
#define IMU_WRAPPER_h

#include "types.h"
#include "ism330.h"
#include "rtos_utils.h"

class IMUWrapper {
    public:
        IMUWrapper() {};
        esp_err_t init(i2c_master_bus_handle_t bus);
        esp_err_t update(const float dt_s, SharedData& out_data);
        Vec3 target_angles = Vec3 { 0.0f, 0.0f, 0.0f };

    private:
		Ism330 imu_;
		IMU::EulerAngles angles;
};

#endif