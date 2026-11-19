#ifndef STABILIZER_h
#define STABILIZER_h

#include "esp_err.h"
#include "types.h"
#include "ism330.h"
#include "imu_sample.h"
#include "imu_mailbox.h"

// IMU wrapper. Reads the ISM330 on its own task and publishes the latest
// ImuSample (roll/pitch in rad, gyro in rad/s) to a mailbox.
// No PID: the policy does the stabilizing.
class Stabilizer {
    public:
        struct Config {
            float mahony_kp      = 0.5f;    // TODO: tune between 0.1 and 1.0
            int   sample_rate_hz = 200;    // fusion rate AND task rate (they must match)
            int   calib_samples  = 200;    // gyro bias calibration, robot must be still
        };

        explicit Stabilizer(ImuMailbox& mailbox, const Config& config);

        void init(i2c_master_bus_handle_t bus);

        // Read the IMU once and publish to the mailbox. Call at sample_rate_hz.
        void update();

        // FreeRTOS task entry: xTaskCreate(Stabilizer::taskEntry, "imu", 4096, &stabilizer, prio, nullptr);
        static void taskEntry(void* self);

        uint32_t errorCount() const { return error_count_; }

    private:
        ImuMailbox& mailbox_;
        Config cfg_;
        Ism330 imu_;
        Imu::EulerAngles angles_deg_{};
        Vec3 gyro_dps_ = Vec3::Zero();
        uint32_t error_count_ = 0;
};

#endif