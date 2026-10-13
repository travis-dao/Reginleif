#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ism330.h"

namespace ism330 {

namespace {

const char *const TAG = "ism330";

/* Registers */
constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t REG_CTRL1_XL = 0x10;
constexpr uint8_t REG_CTRL2_G  = 0x11;
constexpr uint8_t REG_CTRL3_C  = 0x12;
constexpr uint8_t REG_CTRL6_C  = 0x15;
constexpr uint8_t REG_OUT_TEMP_L = 0x20;   /* burst: TEMP (2), GYRO X/Y/Z (6), ACCEL X/Y/Z (6) */

/* CTRL3_C bits */
constexpr uint8_t CTRL3_SWRST  = 1 << 0;
constexpr uint8_t CTRL3_IF_INC = 1 << 2;   /* register auto-increment */
constexpr uint8_t CTRL3_BDU    = 1 << 6;   /* block data update: LSB/MSB always from the same sample */

/* CTRL1_XL bits */
constexpr uint8_t XL_LPF2_EN = 1 << 1;

constexpr uint8_t  CHIP_ID      = 0x6B;
constexpr uint8_t  CTRL6_C_LPF  = 0x03;    /* gyro low-pass filter setting, as in the original driver */
constexpr uint32_t RESET_DELAY_MS = 100;

/* Field encodings, indexed by the enum values */
constexpr uint8_t ACCEL_FS_BITS[] = { 0x00, 0x08, 0x0C, 0x04 };                    /* 2, 4, 8, 16 g (FS_XL[1:0] << 2) */
constexpr float   ACCEL_G_PER_LSB[] = { 0.061e-3f, 0.122e-3f, 0.244e-3f, 0.488e-3f };

constexpr uint8_t GYRO_FS_BITS[] = { 0x02, 0x00, 0x04, 0x08, 0x0C, 0x01 };         /* 125, 250, 500, 1000, 2000, 4000 dps */
constexpr float   GYRO_DPS_PER_LSB[] = { 4.375e-3f, 8.75e-3f, 17.5e-3f, 35.0e-3f, 70.0e-3f, 140.0e-3f };

/* Empirical correction carried over from the original driver ("* 1.5 for scaling error").
   If you added it to compensate for something else, set it to 1.0f. */
constexpr float GYRO_SCALE_CORRECTION = 1.5f;

/* ODR register code (1..10) -> Hz, rounded */
constexpr uint16_t ODR_HZ[] = { 0, 13, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 };

constexpr float DEG2RAD = 0.01745329252f;
constexpr float RAD2DEG = 57.2957795f;

constexpr float MAX_DT_S    = 0.5f;   /* a longer gap means a stall: reseed instead of integrating */
constexpr float ACCEL_MIN_G = 0.5f;   /* outside this window the accel is dominated by motion, */
constexpr float ACCEL_MAX_G = 1.5f;   /* so it is a poor gravity reference and is ignored      */

constexpr uint16_t DISCARD_SAMPLES = 50;   /* startup samples dropped before calibrating */

inline int16_t le16(const uint8_t *p)
{
    return static_cast<int16_t>((static_cast<uint16_t>(p[1]) << 8) | p[0]);
}

inline float invSqrt(float x) { return 1.0f / std::sqrt(x); }

inline bool accelValid(const Data &d)
{
    const float norm = std::sqrt(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
    return norm > ACCEL_MIN_G && norm < ACCEL_MAX_G;
}

inline TickType_t periodTicks(uint16_t hz)
{
    return pdMS_TO_TICKS(1000 / (hz ? hz : 100)) + 1;
}

/* Quaternion for the roll/pitch implied by gravity, yaw = 0 */
void quatFromAccel(const Data &d, float &q0, float &q1, float &q2, float &q3)
{
    const float norm = std::sqrt(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
    const float ax = d.ax / norm, ay = d.ay / norm, az = d.az / norm;

    const float pitch = std::asin(std::clamp(-ax, -1.0f, 1.0f));
    const float roll  = std::atan2(ay, az);

    const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    const float cr = std::cos(roll  * 0.5f), sr = std::sin(roll  * 0.5f);

    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;
}

void quatToEuler(float q0, float q1, float q2, float q3, float &roll, float &pitch, float &yaw)
{
    roll  = std::atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * RAD2DEG;
    pitch = std::asin(std::clamp(-2.0f * (q1 * q3 - q0 * q2), -1.0f, 1.0f)) * RAD2DEG;
    yaw   = std::atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * RAD2DEG;
}

/* Time the filter since its last call, update it, and copy out the angles */
template <typename Filter>
void stepFilter(Filter &f, const Data &d, Angles &out)
{
    const int64_t now = esp_timer_get_time();
    const float dt = f.lastUs() ? (now - f.lastUs()) / 1e6f : 0.0f;
    f.setLastUs(now);

    f.update(d, dt);
    out.roll  = f.roll();
    out.pitch = f.pitch();
    out.yaw   = f.yaw();
}

}  // namespace

/* ---------- Mahony filter ---------- */

void MahonyFilter::setGains(float kp, float ki)
{
    two_kp_ = 2.0f * std::max(kp, 0.0f);
    two_ki_ = 2.0f * std::max(ki, 0.0f);
}

void MahonyFilter::reset()
{
    q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
    ix_ = iy_ = iz_ = 0.0f;
    roll_ = pitch_ = yaw_ = 0.0f;
    seeded_  = false;
    last_us_ = 0;
}

void MahonyFilter::seedFromAccel(const Data &d)
{
    quatFromAccel(d, q0_, q1_, q2_, q3_);
    ix_ = iy_ = iz_ = 0.0f;
    seeded_ = true;
    updateAngles();
}

void MahonyFilter::updateAngles()
{
    quatToEuler(q0_, q1_, q2_, q3_, roll_, pitch_, yaw_);
}

void MahonyFilter::update(const Data &d, float dt)
{
    const bool accel_ok = accelValid(d);

    /* First sample, or a long gap: start from the accelerometer */
    if (!seeded_ || dt <= 0.0f || dt > MAX_DT_S) {
        if (accel_ok) {
            seedFromAccel(d);
        }
        return;
    }

    float gx = d.gx * DEG2RAD;
    float gy = d.gy * DEG2RAD;
    float gz = d.gz * DEG2RAD;

    if (accel_ok) {
        const float r = invSqrt(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
        const float ax = d.ax * r, ay = d.ay * r, az = d.az * r;

        /* Estimated direction of gravity */
        const float halfvx = q1_ * q3_ - q0_ * q2_;
        const float halfvy = q0_ * q1_ + q2_ * q3_;
        const float halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;

        /* Error: cross product between estimated and measured gravity */
        const float halfex = ay * halfvz - az * halfvy;
        const float halfey = az * halfvx - ax * halfvz;
        const float halfez = ax * halfvy - ay * halfvx;

        if (two_ki_ > 0.0f) {
            ix_ += two_ki_ * halfex * dt;
            iy_ += two_ki_ * halfey * dt;
            iz_ += two_ki_ * halfez * dt;
            gx += ix_;
            gy += iy_;
            gz += iz_;
        } else {
            ix_ = iy_ = iz_ = 0.0f;   /* prevent integral windup */
        }

        gx += two_kp_ * halfex;
        gy += two_kp_ * halfey;
        gz += two_kp_ * halfez;
    }

    /* Integrate rate of change of quaternion */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    const float qa = q0_, qb = q1_, qc = q2_;
    q0_ += (-qb * gx - qc * gy - q3_ * gz);
    q1_ += ( qa * gx + qc * gz - q3_ * gy);
    q2_ += ( qa * gy - qb * gz + q3_ * gx);
    q3_ += ( qa * gz + qb * gy - qc * gx);

    const float rn = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= rn; q1_ *= rn; q2_ *= rn; q3_ *= rn;

    updateAngles();
}

/* ---------- Madgwick filter ---------- */

void MadgwickFilter::setBeta(float beta)
{
    beta_ = std::max(beta, 0.0f);
}

void MadgwickFilter::reset()
{
    q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
    roll_ = pitch_ = yaw_ = 0.0f;
    seeded_  = false;
    last_us_ = 0;
}

void MadgwickFilter::seedFromAccel(const Data &d)
{
    quatFromAccel(d, q0_, q1_, q2_, q3_);
    seeded_ = true;
    updateAngles();
}

void MadgwickFilter::updateAngles()
{
    quatToEuler(q0_, q1_, q2_, q3_, roll_, pitch_, yaw_);
}

void MadgwickFilter::update(const Data &d, float dt)
{
    const bool accel_ok = accelValid(d);

    if (!seeded_ || dt <= 0.0f || dt > MAX_DT_S) {
        if (accel_ok) {
            seedFromAccel(d);
        }
        return;
    }

    const float gx = d.gx * DEG2RAD;
    const float gy = d.gy * DEG2RAD;
    const float gz = d.gz * DEG2RAD;

    /* Rate of change of quaternion from gyroscope */
    float qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
    float qDot2 = 0.5f * ( q0_ * gx + q2_ * gz - q3_ * gy);
    float qDot3 = 0.5f * ( q0_ * gy - q1_ * gz + q3_ * gx);
    float qDot4 = 0.5f * ( q0_ * gz + q1_ * gy - q2_ * gx);

    if (accel_ok) {
        const float r = invSqrt(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
        const float ax = d.ax * r, ay = d.ay * r, az = d.az * r;

        const float _2q0 = 2.0f * q0_, _2q1 = 2.0f * q1_, _2q2 = 2.0f * q2_, _2q3 = 2.0f * q3_;
        const float _4q0 = 4.0f * q0_, _4q1 = 4.0f * q1_, _4q2 = 4.0f * q2_;
        const float _8q1 = 8.0f * q1_, _8q2 = 8.0f * q2_;
        const float q0q0 = q0_ * q0_, q1q1 = q1_ * q1_, q2q2 = q2_ * q2_, q3q3 = q3_ * q3_;

        /* Gradient descent corrective step */
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay - _4q1 +
                   _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
                   _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;

        const float sn = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (sn > 0.0f) {   /* zero when already aligned: skip to avoid dividing by 0 */
            const float rs = invSqrt(sn);
            s0 *= rs; s1 *= rs; s2 *= rs; s3 *= rs;

            qDot1 -= beta_ * s0;
            qDot2 -= beta_ * s1;
            qDot3 -= beta_ * s2;
            qDot4 -= beta_ * s3;
        }
    }

    q0_ += qDot1 * dt;
    q1_ += qDot2 * dt;
    q2_ += qDot3 * dt;
    q3_ += qDot4 * dt;

    const float rn = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= rn; q1_ *= rn; q2_ *= rn; q3_ *= rn;

    updateAngles();
}

/* ---------- lifecycle ---------- */

Ism330::~Ism330()
{
    if (i2c_) {
        deinit();
    }
}

esp_err_t Ism330::init(const Config &cfg)
{
    ESP_RETURN_ON_FALSE(cfg.bus, ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_FALSE(!i2c_, ESP_ERR_INVALID_STATE, TAG, "already initialized");

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = cfg.addr ? cfg.addr : ISM_I2C_ADDR_DEFAULT;
    dev_cfg.scl_speed_hz    = cfg.scl_speed_hz ? cfg.scl_speed_hz : 400000;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(cfg.bus, &dev_cfg, &i2c_), TAG, "add device failed");

    esp_err_t ret = configure(cfg);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(i2c_);
        i2c_ = nullptr;
        return ret;
    }

    ESP_LOGI(TAG, "ISM330DHCX ready @0x%02X", static_cast<unsigned>(dev_cfg.device_address));
    return ESP_OK;
}

/* Everything after the I2C device is attached; the caller cleans up on failure. */
esp_err_t Ism330::configure(const Config &cfg)
{
    /* Identify the chip */
    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(readRegs(REG_WHO_AM_I, &who, 1), TAG, "WHO_AM_I read failed (check wiring/address)");
    if (who != CHIP_ID) {
        ESP_LOGE(TAG, "unexpected WHO_AM_I 0x%02X", who);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(reset(), TAG, "reset failed");
    ESP_RETURN_ON_ERROR(writeReg(REG_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC), TAG, "ctrl3");

    /* Reset all state to defaults, then apply configuration */
    for (int i = 0; i < 3; i++) {
        accel_bias_[i] = 0.0f;
        gyro_bias_[i]  = 0.0f;
    }
    sample_rate_hz_ = 0;

    ESP_RETURN_ON_ERROR(setAccelFs(cfg.accel_fs), TAG, "accel fs");
    ESP_RETURN_ON_ERROR(setGyroFs(cfg.gyro_fs), TAG, "gyro fs");
    ESP_RETURN_ON_ERROR(setOdr(cfg.odr), TAG, "odr");
    ESP_RETURN_ON_ERROR(updateReg(REG_CTRL1_XL, XL_LPF2_EN, XL_LPF2_EN), TAG, "accel lpf2");
    ESP_RETURN_ON_ERROR(writeReg(REG_CTRL6_C, CTRL6_C_LPF), TAG, "gyro lpf");

    if (cfg.sample_rate_hz) {
        sample_rate_hz_ = cfg.sample_rate_hz;
    }

    fusion_   = cfg.fusion;
    mahony_   = MahonyFilter(cfg.mahony_kp, cfg.mahony_ki);
    madgwick_ = MadgwickFilter(cfg.madgwick_beta);
    return ESP_OK;
}

esp_err_t Ism330::deinit()
{
    ESP_RETURN_ON_FALSE(i2c_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    /* Best effort: power both sensors down */
    updateReg(REG_CTRL1_XL, 0xF0, 0);
    updateReg(REG_CTRL2_G, 0xF0, 0);
    esp_err_t ret = i2c_master_bus_rm_device(i2c_);
    i2c_ = nullptr;
    return ret;
}

esp_err_t Ism330::reset()
{
    ESP_RETURN_ON_FALSE(i2c_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_ERROR(writeReg(REG_CTRL3_C, CTRL3_SWRST), TAG, "SWRST write failed");
    vTaskDelay(pdMS_TO_TICKS(RESET_DELAY_MS));
    return ESP_OK;
}

/* ---------- low-level access ---------- */

esp_err_t Ism330::readRegs(uint8_t reg, uint8_t *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(i2c_ && buf && len, ESP_ERR_INVALID_ARG, TAG, "bad args");
    return i2c_master_transmit_receive(i2c_, &reg, 1, buf, len, 100);
}

esp_err_t Ism330::writeReg(uint8_t reg, uint8_t val)
{
    ESP_RETURN_ON_FALSE(i2c_, ESP_ERR_INVALID_ARG, TAG, "bad args");
    const uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(i2c_, buf, sizeof(buf), 100);
}

/* Read-modify-write helper */
esp_err_t Ism330::updateReg(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t cur;
    ESP_RETURN_ON_ERROR(readRegs(reg, &cur, 1), TAG, "read reg 0x%02X", reg);
    cur = static_cast<uint8_t>((cur & ~mask) | (val & mask));
    return writeReg(reg, cur);
}

/* ---------- configuration ---------- */

esp_err_t Ism330::setAccelFs(AccelFs fs)
{
    const uint8_t idx = static_cast<uint8_t>(fs);
    ESP_RETURN_ON_FALSE(i2c_ && idx <= static_cast<uint8_t>(AccelFs::G16), ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_ERROR(updateReg(REG_CTRL1_XL, 0x0C, ACCEL_FS_BITS[idx]), TAG, "accel cfg");
    accel_fs_ = fs;
    accel_g_per_lsb_ = ACCEL_G_PER_LSB[idx];
    return ESP_OK;
}

esp_err_t Ism330::setGyroFs(GyroFs fs)
{
    const uint8_t idx = static_cast<uint8_t>(fs);
    ESP_RETURN_ON_FALSE(i2c_ && idx <= static_cast<uint8_t>(GyroFs::Dps4000), ESP_ERR_INVALID_ARG, TAG, "bad args");
    /* FS_G[1:0] (bits 3:2), FS_125 (bit 1), FS_4000 (bit 0) */
    ESP_RETURN_ON_ERROR(updateReg(REG_CTRL2_G, 0x0F, GYRO_FS_BITS[idx]), TAG, "gyro cfg");
    gyro_fs_ = fs;
    gyro_dps_per_lsb_ = GYRO_DPS_PER_LSB[idx] * GYRO_SCALE_CORRECTION;
    return ESP_OK;
}

esp_err_t Ism330::setOdr(Odr odr)
{
    const uint8_t code = static_cast<uint8_t>(odr);
    ESP_RETURN_ON_FALSE(i2c_ && code >= static_cast<uint8_t>(Odr::Hz12_5) &&
                        code <= static_cast<uint8_t>(Odr::Hz6667), ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_ERROR(updateReg(REG_CTRL1_XL, 0xF0, static_cast<uint8_t>(code << 4)), TAG, "accel odr");
    ESP_RETURN_ON_ERROR(updateReg(REG_CTRL2_G,  0xF0, static_cast<uint8_t>(code << 4)), TAG, "gyro odr");
    odr_ = odr;
    sample_rate_hz_ = ODR_HZ[code];
    return ESP_OK;
}

/* ---------- data ---------- */

esp_err_t Ism330::readRaw(Raw &raw)
{
    /* One burst keeps the samples coherent */
    uint8_t b[14];
    ESP_RETURN_ON_ERROR(readRegs(REG_OUT_TEMP_L, b, sizeof(b)), TAG, "burst read");

    raw.temp = le16(&b[0]);
    raw.gx   = le16(&b[2]);
    raw.gy   = le16(&b[4]);
    raw.gz   = le16(&b[6]);
    raw.ax   = le16(&b[8]);
    raw.ay   = le16(&b[10]);
    raw.az   = le16(&b[12]);
    return ESP_OK;
}

esp_err_t Ism330::read(Data &d)
{
    Raw r;
    ESP_RETURN_ON_ERROR(readRaw(r), TAG, "read raw");

    /* 256 LSB/degC, reads 0 at 25 degC */
    const float t = r.temp / 256.0f;

    const float as = 1.0f + accelTempScale(t), ao = accelTempOffset(t);
    const float gs = 1.0f + gyroTempScale(t),  go = gyroTempOffset(t);

    d.ax = ((r.ax * accel_g_per_lsb_ - accel_bias_[0]) + ao) * as;
    d.ay = ((r.ay * accel_g_per_lsb_ - accel_bias_[1]) + ao) * as;
    d.az = ((r.az * accel_g_per_lsb_ - accel_bias_[2]) + ao) * as;
    d.gx = ((r.gx * gyro_dps_per_lsb_ - gyro_bias_[0]) + go) * gs;
    d.gy = ((r.gy * gyro_dps_per_lsb_ - gyro_bias_[1]) + go) * gs;
    d.gz = ((r.gz * gyro_dps_per_lsb_ - gyro_bias_[2]) + go) * gs;
    d.temp_c = t + 25.0f;
    return ESP_OK;
}

esp_err_t Ism330::readAccel(Vector &accel)
{
    Data d;
    ESP_RETURN_ON_ERROR(read(d), TAG, "read");
    accel = { d.ax, d.ay, d.az };
    return ESP_OK;
}

esp_err_t Ism330::readGyro(Vector &gyro)
{
    Data d;
    ESP_RETURN_ON_ERROR(read(d), TAG, "read");
    gyro = { d.gx, d.gy, d.gz };
    return ESP_OK;
}

esp_err_t Ism330::readVectors(Vector &accel, Vector &gyro)
{
    Data d;
    ESP_RETURN_ON_ERROR(read(d), TAG, "read");
    accel = { d.ax, d.ay, d.az };
    gyro  = { d.gx, d.gy, d.gz };
    return ESP_OK;
}

esp_err_t Ism330::readTemperature(float &temp_offset_c)
{
    uint8_t b[2];
    ESP_RETURN_ON_ERROR(readRegs(REG_OUT_TEMP_L, b, sizeof(b)), TAG, "temp read");
    temp_offset_c = le16(b) / 256.0f;
    return ESP_OK;
}

/* ---------- temperature compensation (argument: offset from 25 degC) ---------- */

float Ism330::accelTempScale(float temp) const  { return temp * 0.00005f; }   /* 0.005 %/degC */
float Ism330::accelTempOffset(float temp) const { return temp * 0.0001f; }    /* 0.1 mg/degC  */
float Ism330::gyroTempScale(float temp) const   { return temp * 0.00007f; }   /* 0.007 %/degC */
float Ism330::gyroTempOffset(float temp) const  { return temp * 0.005f; }     /* 0.005 dps/degC */

/* ---------- calibration ---------- */

esp_err_t Ism330::calibrateGyro(uint16_t samples)
{
    ESP_RETURN_ON_FALSE(i2c_ && samples, ESP_ERR_INVALID_ARG, TAG, "bad args");

    const TickType_t period = periodTicks(sample_rate_hz_);
    Raw r;

    /* Drop the first samples: the sensor is still settling after power-up */
    for (uint16_t i = 0; i < DISCARD_SAMPLES; i++) {
        ESP_RETURN_ON_ERROR(readRaw(r), TAG, "read raw");
        vTaskDelay(period);
    }

    int64_t sum[3] = {0, 0, 0};
    for (uint16_t i = 0; i < samples; i++) {
        ESP_RETURN_ON_ERROR(readRaw(r), TAG, "read raw");
        sum[0] += r.gx;
        sum[1] += r.gy;
        sum[2] += r.gz;
        vTaskDelay(period);
    }
    for (int i = 0; i < 3; i++) {
        gyro_bias_[i] = (static_cast<float>(sum[i]) / samples) * gyro_dps_per_lsb_;
    }
    ESP_LOGI(TAG, "gyro bias: %.3f %.3f %.3f dps", gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);

    resetAngles();   /* drop stale filter state */
    return ESP_OK;
}

esp_err_t Ism330::calibrateAccel(uint16_t samples)
{
    ESP_RETURN_ON_FALSE(i2c_ && samples, ESP_ERR_INVALID_ARG, TAG, "bad args");

    const TickType_t period = periodTicks(sample_rate_hz_);
    Raw r;

    for (uint16_t i = 0; i < DISCARD_SAMPLES; i++) {
        ESP_RETURN_ON_ERROR(readRaw(r), TAG, "read raw");
        vTaskDelay(period);
    }

    float sum[3] = {0.0f, 0.0f, 0.0f};
    for (uint16_t i = 0; i < samples; i++) {
        ESP_RETURN_ON_ERROR(readRaw(r), TAG, "read raw");
        sum[0] += r.ax * accel_g_per_lsb_;
        sum[1] += r.ay * accel_g_per_lsb_;
        sum[2] += r.az * accel_g_per_lsb_;
        vTaskDelay(period);
    }

    /* Level board: x = y = 0 g, z = 1 g */
    accel_bias_[0] = sum[0] / samples;
    accel_bias_[1] = sum[1] / samples;
    accel_bias_[2] = sum[2] / samples - 1.0f;
    ESP_LOGI(TAG, "accel bias: %.4f %.4f %.4f g", accel_bias_[0], accel_bias_[1], accel_bias_[2]);

    resetAngles();
    return ESP_OK;
}

esp_err_t Ism330::calibrate(uint16_t samples)
{
    ESP_RETURN_ON_ERROR(calibrateGyro(samples), TAG, "gyro calibration");
    ESP_RETURN_ON_ERROR(calibrateAccel(samples), TAG, "accel calibration");
    return ESP_OK;
}

/* ---------- angle estimate ---------- */

esp_err_t Ism330::getAngles(Angles &angles)
{
    ESP_RETURN_ON_FALSE(i2c_, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    Data d;
    ESP_RETURN_ON_ERROR(read(d), TAG, "read");

    if (fusion_ == FusionType::Mahony) {
        stepFilter(mahony_, d, angles);
    } else {
        stepFilter(madgwick_, d, angles);
    }
    return ESP_OK;
}

void Ism330::resetAngles()
{
    mahony_.reset();
    madgwick_.reset();
}

void Ism330::setFusion(FusionType type)
{
    if (type != fusion_) {
        /* The newly selected filter has been idle: make it reseed from the accelerometer */
        if (type == FusionType::Mahony) {
            mahony_.reset();
        } else {
            madgwick_.reset();
        }
        fusion_ = type;
    }
}

void Ism330::setMahonyGains(float kp, float ki)
{
    mahony_.setGains(kp, ki);
}

esp_err_t Ism330::setMadgwickBeta(float beta)
{
    ESP_RETURN_ON_FALSE(beta >= 0.0f, ESP_ERR_INVALID_ARG, TAG, "bad args");
    madgwick_.setBeta(beta);
    return ESP_OK;
}

}