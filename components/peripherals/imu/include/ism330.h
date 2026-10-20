/**
 * @file ism330.h
 * @brief I2C driver for the ISM330DHCX 6-DoF accelerometer + gyroscope (ESP-IDF, i2c_master API).
 *
 * init() attaches the device, verifies WHO_AM_I, resets the chip and applies the
 * configuration (full-scale ranges, output IMU::data rate), so the sensor no longer
 * needs to be configured externally before sampling.
 */
#pragma once

#include "driver/i2c_master.h"
#include "imu.h"
#include "types.h"

constexpr uint8_t ISM_I2C_ADDR_DEFAULT = 0x6A;   /* SDO/SA0 low; 0x6B when high */

/* ---------- sensor fusion filters ---------- */

/**
 * Mahony AHRS (6-axis: accel + gyro). Quaternion state, seeded from the
 * accelerometer on the first valid sample or after a long gap.
 */
class MahonyFilter {
    public:
        explicit MahonyFilter(float kp = 0.1f, float ki = 0.0f) { setGains(kp, ki); }

        void setGains(float kp, float ki);
        void reset();
        void update(const IMU::Data &d, float dt);          /* d.g* in deg/s, d.a* in g */

        float roll()  const { return roll_; }
        float pitch() const { return pitch_; }
        float yaw()   const { return yaw_; }

        int64_t lastUs() const { return last_us_; }
        void    setLastUs(int64_t us) { last_us_ = us; }

    private:
        void seedFromAccel(const IMU::Data &d);
        void updateAngles();

        float two_kp_ = 0.2f, two_ki_ = 0.0f;
        float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
        float ix_ = 0.0f, iy_ = 0.0f, iz_ = 0.0f;     /* integral error terms scaled by Ki */
        float roll_ = 0.0f, pitch_ = 0.0f, yaw_ = 0.0f;
        bool    seeded_  = false;
        int64_t last_us_ = 0;
};

/** Madgwick AHRS (6-axis: accel + gyro). */
class MadgwickFilter {
    public:
        explicit MadgwickFilter(float beta = 0.1f) { setBeta(beta); }

        void setBeta(float beta);
        void reset();
        void update(const IMU::Data &d, float dt);

        float roll()  const { return roll_; }
        float pitch() const { return pitch_; }
        float yaw()   const { return yaw_; }

        int64_t lastUs() const { return last_us_; }
        void    setLastUs(int64_t us) { last_us_ = us; }

    private:
        void seedFromAccel(const IMU::Data &d);
        void updateAngles();

        float beta_ = 0.1f;
        float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
        float roll_ = 0.0f, pitch_ = 0.0f, yaw_ = 0.0f;
        bool    seeded_  = false;
        int64_t last_us_ = 0;
};

/* ---------- driver ---------- */

class Ism330: public IMU {
    public:
        // config enums
        enum class AccelFs : uint8_t { G2 = 0, G4, G8, G16 };
        enum class GyroFs  : uint8_t { Dps125 = 0, Dps250, Dps500, Dps1000, Dps2000, Dps4000 };

        /* Output IMU::data rate, applied to both accel and gyro */
        enum class Odr : uint8_t {
            Hz12_5 = 1, Hz26, Hz52, Hz104, Hz208, Hz417, Hz833, Hz1667, Hz3333, Hz6667
        };

        enum class FusionType : uint8_t { Mahony, Madgwick };

        struct Config {
            i2c_master_bus_handle_t bus = nullptr;   /**< Existing I2C master bus (required)       */
            uint8_t  addr          = 0;              /**< 7-bit address, 0 -> 0x6A                 */
            uint32_t scl_speed_hz  = 0;              /**< 0 -> 400 kHz                             */
            AccelFs  accel_fs      = AccelFs::G4;
            GyroFs   gyro_fs       = GyroFs::Dps500;
            Odr      odr           = Odr::Hz104;
            uint16_t sample_rate_hz = 0;             /**< expected getAngles() rate; 0 -> match ODR */
            FusionType fusion      = FusionType::Mahony;
            float    mahony_kp     = 0.1f;           /**< proportional gain                        */
            float    mahony_ki     = 0.0f;           /**< integral gain                            */
            float    madgwick_beta = 0.1f;           /**< algorithm gain                           */
        };
        

        Ism330() = default;
        ~Ism330();

        /* Owns an I2C device handle: not copyable */
        Ism330(const Ism330 &)            = delete;
        Ism330 &operator=(const Ism330 &) = delete;

        /** Attach to the bus, verify WHO_AM_I, reset and configure the chip. */
        esp_err_t init(const Config &cfg);
        esp_err_t deinit();

        /** Software reset; blocks briefly while the device restarts. Config must be re-applied. */
        esp_err_t reset();

        bool initialized() const { return i2c_ != nullptr; }

        /* --- configuration --- */
        esp_err_t setAccelFs(AccelFs fs);
        esp_err_t setGyroFs(GyroFs fs);
        esp_err_t setOdr(Odr odr);

        /* --- IMU::data --- */
        /** One burst read of temperature, gyro and accel. Keeps the samples coherent. */
        esp_err_t readRaw(Raw &raw);

        /** Scaled to g and deg/s, with biases subtracted. */
        esp_err_t read(Data &d);

        esp_err_t readAccel(Vec3 &accel);
        esp_err_t readGyro(Vec3 &gyro);
        esp_err_t readVectors(Vec3 &accel, Vec3 &gyro);   /* replaces getAGVectors() */

        /** Temperature relative to 25 C (reads 0 at 25 C): this is how the chip is calibrated. */
        esp_err_t readTemperature(float &temp_offset_c);

        /* Temperature compensation terms. Argument is the offset from 25 C (0 at 25 C). */
        float accelTempScale(float temp) const;
        float accelTempOffset(float temp) const;   /* constant added to the reading */
        float gyroTempScale(float temp) const;
        float gyroTempOffset(float temp) const;    /* constant added to the reading */

        /* --- calibration (keep the board still; level for calibrateAccel) --- */
        esp_err_t calibrateGyro(uint16_t samples = 200);
        esp_err_t calibrateAccel(uint16_t samples = 200);   /* assumes level: x = y = 0 g, z = 1 g */
        esp_err_t calibrate(uint16_t samples = 200);        /* gyro, then accel */

        /* --- angle estimate (call at a steady rate) --- */
        /** Reads the sensor, runs the selected filter and returns the Euler angles. */
        esp_err_t getAngles(EulerAngles &angles);

        void resetAngles();
        void setFusion(FusionType type);
        void setMahonyGains(float kp, float ki);
        esp_err_t setMadgwickBeta(float beta);

    private:
        esp_err_t configure(const Config &cfg);

        esp_err_t readRegs(uint8_t reg, uint8_t *buf, size_t len);
        esp_err_t writeReg(uint8_t reg, uint8_t val);
        esp_err_t updateReg(uint8_t reg, uint8_t mask, uint8_t val);

        i2c_master_dev_handle_t i2c_ = nullptr;

        AccelFs accel_fs_ = AccelFs::G4;
        GyroFs  gyro_fs_  = GyroFs::Dps500;
        Odr     odr_      = Odr::Hz104;
        float   accel_g_per_lsb_ = 0.0f;
        float   gyro_dps_per_lsb_ = 0.0f;
        uint16_t sample_rate_hz_ = 0;

        /* Biases removed in read() */
        float accel_bias_[3] = {0.0f, 0.0f, 0.0f};   /* g     */
        float gyro_bias_[3]  = {0.0f, 0.0f, 0.0f};   /* deg/s */

        FusionType     fusion_ = FusionType::Mahony;
        MahonyFilter   mahony_;
        MadgwickFilter madgwick_;
};