/*
 * MPU6050 driver for ESP-IDF (>= v5.2) using the new i2c_master API. (C++ port)
 */
#pragma once

#include "driver/i2c_master.h"
#include "imu.h"

/* 7-bit I2C addresses (selected by the AD0 pin) */
constexpr uint8_t ADDR_AD0_LOW  = 0x68;
constexpr uint8_t ADDR_AD0_HIGH = 0x69;

/* Complementary filter for roll/pitch. Usable standalone or via Mpu6050::getAngles(). */
class ComplementaryFilter {
    public:
        /* alpha: gyro weight, 0..1 (typ. 0.95..0.99). Related to a time constant tau by
        * alpha = tau / (tau + dt); e.g. 0.98 at 200 Hz is tau of about 0.25 s. */
        explicit ComplementaryFilter(float alpha = 0.98f) { setAlpha(alpha); }

        void  setAlpha(float alpha);   /* clamped to 0..1 */
        float alpha() const { return alpha_; }

        /* Clears state (keeps alpha); the next sample reseeds from the accelerometer. */
        void reset();

        /* One filter step on an already-read sample (pure math, no I2C). Use this if you
        * manage timing yourself. The first call seeds the angles from the accelerometer.
        * dt is seconds since the previous update. */
        void update(const IMU::Data &d, float dt);

        float roll() const  { return roll_; }
        float pitch() const { return pitch_; }

        /* Timestamp bookkeeping used by Mpu6050::readAngles() to compute dt. */
        int64_t last_us() const { return last_us_; }
        void    set_last_us(int64_t us) { last_us_ = us; }

    private:
        float   alpha_   = 0.98f;
        float   roll_    = 0.0f;
        float   pitch_   = 0.0f;
        bool    seeded_  = false;
        int64_t last_us_ = 0;
};

class Mpu6050: public IMU {
    public:
        // config enums
        enum class AccelFs : uint8_t { G2 = 0, G4 = 1, G8 = 2, G16 = 3 };
        enum class GyroFs  : uint8_t { Dps250 = 0, Dps500 = 1, Dps1000 = 2, Dps2000 = 3 };

        /* Digital low-pass filter setting (CONFIG register DLPF_CFG).
        * Hz260 disables the filter and raises the gyro output rate to 8 kHz. */
        enum class Dlpf : uint8_t {
            Hz260 = 0,   /* accel 260 Hz / gyro 256 Hz, gyro rate 8 kHz */
            Hz184 = 1,   /* accel 184 Hz / gyro 188 Hz, gyro rate 1 kHz */
            Hz94  = 2,
            Hz44  = 3,
            Hz21  = 4,
            Hz10  = 5,
            Hz5   = 6,
        };
        
        struct Config {
            i2c_master_bus_handle_t bus = nullptr;   /* An already-created I2C master bus */
            uint8_t  addr           = ADDR_AD0_LOW;
            uint32_t scl_speed_hz   = 400000;        /* Up to 400000 (0 -> 400 kHz) */
            AccelFs  accel_fs       = AccelFs::G4;
            GyroFs   gyro_fs        = GyroFs::Dps500;
            Dlpf     dlpf           = Dlpf::Hz44;
            uint16_t sample_rate_hz = 200;           /* see Mpu6050::setSampleRate() (0 -> 100 Hz) */
            float    filter_alpha   = 0.98f;         /* gyro weight for getAngles() (0 -> 0.98) */
        };
        
        Mpu6050() = default;
        ~Mpu6050();   /* puts the chip to sleep and releases the I2C device (bus is NOT deleted) */

        Mpu6050(const Mpu6050 &)            = delete;
        Mpu6050 &operator=(const Mpu6050 &) = delete;

        /* Verify WHO_AM_I, reset, and apply the configuration. Separate from the constructor
        * because ESP-IDF builds without exceptions by default. */
        esp_err_t init(const Config &cfg);
        esp_err_t deinit();
        bool      initialized() const { return i2c_ != nullptr; }

        /* Low-level register access */
        esp_err_t readRegs(uint8_t reg, uint8_t *buf, size_t len);
        esp_err_t writeReg(uint8_t reg, uint8_t val);

        /* Configuration */
        esp_err_t setAccelFs(AccelFs fs);
        esp_err_t setGyroFs(GyroFs fs);
        esp_err_t setDlpf(Dlpf dlpf);
        /* Rate = gyro_output_rate / (1 + SMPLRT_DIV), where gyro_output_rate is 8 kHz for
        * Dlpf::Hz260 and 1 kHz otherwise. Valid: 4..1000 Hz (32..8000 Hz with filter disabled). */
        esp_err_t setSampleRate(uint16_t hz);
        esp_err_t sleep(bool enable);

        /* Data */
        esp_err_t readRaw(Raw &raw);     /* 14 bytes in one burst */
        esp_err_t read(Data &d);         /* converted to physical units */

        /* Keep the board still. Gyro bias is subtracted in read(). */
        esp_err_t calibrateGyro(uint16_t samples);
        esp_err_t calibrateLevel(uint16_t samples);

        /* Angles */
        esp_err_t readAngles(ComplementaryFilter &cf, Data *d = nullptr);   /* d may be nullptr */
        /* Reads the sensor and returns filtered roll/pitch using the built-in filter. Call at a
        * steady rate. Not thread-safe per object. */
        esp_err_t getAngles(EulerAngles &angles);
        esp_err_t setFilterAlpha(float alpha);   /* 0..1 */
        void      resetAngles() { cf_.reset(); } /* next getAngles() reseeds from the accelerometer */

    private:
        esp_err_t configure(const Config &cfg);
        esp_err_t updateReg(uint8_t reg, uint8_t mask, uint8_t val);

        i2c_master_dev_handle_t i2c_ = nullptr;
        AccelFs  accel_fs_ = AccelFs::G2;
        GyroFs   gyro_fs_  = GyroFs::Dps250;
        Dlpf     dlpf_     = Dlpf::Hz260;
        uint16_t sample_rate_hz_ = 0;
        float    accel_lsb_per_g_  = 16384.0f;
        float    gyro_lsb_per_dps_ = 131.0f;
        float    gyro_bias_[3] = {0.0f, 0.0f, 0.0f};   /* deg/s */
        ComplementaryFilter cf_;
        float    roll_off_  = 0.0f;                    /* deg: attitude of the "zero" pose */
        float    pitch_off_ = 0.0f;
};