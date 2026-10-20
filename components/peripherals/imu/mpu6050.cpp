#include <cmath>
#include <cstdlib>
#include <cstring>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"

const char *const TAG = "mpu6050";

/* Registers */
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

/* PWR_MGMT_1 bits */
constexpr uint8_t PWR1_DEVICE_RESET = 1 << 7;
constexpr uint8_t PWR1_SLEEP        = 1 << 6;
constexpr uint8_t PWR1_CLKSEL_PLL_X = 0x01;   /* PLL with X-axis gyro reference: more stable than the internal RC */

constexpr uint8_t WHO_AM_I_MPU6050 = 0x68;
constexpr uint8_t WHO_AM_I_MPU6500 = 0x70;
constexpr uint8_t WHO_AM_I_CLONE   = 0x72;
constexpr uint32_t I2C_TIMEOUT_MS  = 100;

constexpr float RAD2DEG        = 57.2957795f;
constexpr float CF_MAX_DT_S    = 0.5f;   /* a longer gap means a stall: reseed instead of integrating */
constexpr float CF_ACCEL_MIN_G = 0.5f;   /* outside this window the accel is dominated by motion, */
constexpr float CF_ACCEL_MAX_G = 1.5f;   /* so it is a poor gravity reference and is ignored      */

constexpr float ACCEL_SENS[] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
constexpr float GYRO_SENS[]  = { 131.0f, 65.5f, 32.8f, 16.4f };

bool whoAmIOk(uint8_t who)
{
    return who == WHO_AM_I_MPU6050 || who == WHO_AM_I_MPU6500 || who == WHO_AM_I_CLONE;
}

inline int16_t be16(const uint8_t *p)
{
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

/* ---------- complementary filter ---------- */

void ComplementaryFilter::setAlpha(float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    alpha_ = alpha;
}

void ComplementaryFilter::reset()
{
    roll_    = 0.0f;
    pitch_   = 0.0f;
    seeded_  = false;
    last_us_ = 0;
}

void ComplementaryFilter::update(const IMU::Data &d, float dt)
{
    /* Angles implied by gravity alone */
    const float norm = std::sqrt(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
    const bool accel_ok = (norm > CF_ACCEL_MIN_G) && (norm < CF_ACCEL_MAX_G);
    float acc_roll = 0.0f, acc_pitch = 0.0f;
    if (accel_ok) {
        acc_roll  = std::atan2(d.ay, d.az) * RAD2DEG;
        acc_pitch = std::atan2(-d.ax, std::sqrt(d.ay * d.ay + d.az * d.az)) * RAD2DEG;
    }

    /* First sample, or a long gap: start from the accelerometer */
    if (!seeded_ || dt <= 0.0f || dt > CF_MAX_DT_S) {
        if (accel_ok) {
            roll_   = acc_roll;
            pitch_  = acc_pitch;
            seeded_ = true;
        }
        return;
    }

    /* Integrate the gyro (small-angle approximation: body rates used as Euler rates) */
    const float gyro_roll  = roll_  + d.gx * dt;
    const float gyro_pitch = pitch_ + d.gy * dt;

    if (accel_ok) {
        roll_  = alpha_ * gyro_roll  + (1.0f - alpha_) * acc_roll;
        pitch_ = alpha_ * gyro_pitch + (1.0f - alpha_) * acc_pitch;
    } else {
        roll_  = gyro_roll;     /* gyro only while the accel is unreliable */
        pitch_ = gyro_pitch;
    }
}

/* ---------- lifecycle ---------- */

Mpu6050::~Mpu6050()
{
    if (i2c_) {
        deinit();
    }
}

esp_err_t Mpu6050::init(const Config &cfg)
{
    ESP_RETURN_ON_FALSE(cfg.bus, ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_FALSE(!i2c_, ESP_ERR_INVALID_STATE, TAG, "already initialized");

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = cfg.addr;
    dev_cfg.scl_speed_hz    = cfg.scl_speed_hz ? cfg.scl_speed_hz : 400000;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(cfg.bus, &dev_cfg, &i2c_), TAG, "add device failed");

    esp_err_t ret = configure(cfg);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(i2c_);
        i2c_ = nullptr;
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 ready @0x%02X", cfg.addr);
    return ESP_OK;
}

/* Everything after the I2C device is attached; the caller cleans up on failure. */
esp_err_t Mpu6050::configure(const Config &cfg)
{
    /* Identify the chip */
    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(readRegs(REG_WHO_AM_I, &who, 1), TAG, "WHO_AM_I read failed (check wiring/address)");
    if (!whoAmIOk(who)) {
        ESP_LOGE(TAG, "unexpected WHO_AM_I 0x%02X", who);
        return ESP_ERR_NOT_FOUND;
    }
    if (who != WHO_AM_I_MPU6050) {
        ESP_LOGW(TAG, "WHO_AM_I 0x%02X: not a genuine MPU6050, running in compatible mode", who);
    }

    /* Reset, then wake up with the gyro PLL as clock source */
    ESP_RETURN_ON_ERROR(writeReg(REG_PWR_MGMT_1, PWR1_DEVICE_RESET), TAG, "reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(writeReg(REG_PWR_MGMT_1, PWR1_CLKSEL_PLL_X), TAG, "wake failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Reset all state to defaults, then apply configuration
       (DLPF first: the sample rate base depends on it) */
    gyro_bias_[0] = gyro_bias_[1] = gyro_bias_[2] = 0.0f;
    roll_off_ = pitch_off_ = 0.0f;
    sample_rate_hz_ = 0;
    dlpf_ = Dlpf::Hz260;   /* reset value, keeps set_dlpf() bookkeeping right */

    ESP_RETURN_ON_ERROR(setDlpf(cfg.dlpf), TAG, "dlpf");
    ESP_RETURN_ON_ERROR(setAccelFs(cfg.accel_fs), TAG, "accel fs");
    ESP_RETURN_ON_ERROR(setGyroFs(cfg.gyro_fs), TAG, "gyro fs");
    ESP_RETURN_ON_ERROR(setSampleRate(cfg.sample_rate_hz ? cfg.sample_rate_hz : 100), TAG, "sample rate");

    cf_ = ComplementaryFilter(cfg.filter_alpha > 0.0f ? cfg.filter_alpha : 0.98f);
    return ESP_OK;
}

esp_err_t Mpu6050::deinit()
{
    ESP_RETURN_ON_FALSE(i2c_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    sleep(true);   /* best effort */
    esp_err_t ret = i2c_master_bus_rm_device(i2c_);
    i2c_ = nullptr;
    return ret;
}

/* ---------- low-level access ---------- */

esp_err_t Mpu6050::readRegs(uint8_t reg, uint8_t *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(i2c_ && buf && len, ESP_ERR_INVALID_ARG, TAG, "bad args");
    return i2c_master_transmit_receive(i2c_, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

esp_err_t Mpu6050::writeReg(uint8_t reg, uint8_t val)
{
    ESP_RETURN_ON_FALSE(i2c_, ESP_ERR_INVALID_ARG, TAG, "bad args");
    const uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(i2c_, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

/* Read-modify-write helper */
esp_err_t Mpu6050::updateReg(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t cur;
    ESP_RETURN_ON_ERROR(readRegs(reg, &cur, 1), TAG, "read reg 0x%02X", reg);
    cur = static_cast<uint8_t>((cur & ~mask) | (val & mask));
    return writeReg(reg, cur);
}

/* ---------- configuration ---------- */

esp_err_t Mpu6050::setAccelFs(AccelFs fs)
{
    const uint8_t idx = static_cast<uint8_t>(fs);
    ESP_RETURN_ON_FALSE(i2c_ && idx <= static_cast<uint8_t>(AccelFs::G16), ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_ERROR(writeReg(REG_ACCEL_CONFIG, static_cast<uint8_t>(idx << 3)), TAG, "accel cfg");
    accel_fs_ = fs;
    accel_lsb_per_g_ = ACCEL_SENS[idx];
    return ESP_OK;
}

esp_err_t Mpu6050::setGyroFs(GyroFs fs)
{
    const uint8_t idx = static_cast<uint8_t>(fs);
    ESP_RETURN_ON_FALSE(i2c_ && idx <= static_cast<uint8_t>(GyroFs::Dps2000), ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_ERROR(writeReg(REG_GYRO_CONFIG, static_cast<uint8_t>(idx << 3)), TAG, "gyro cfg");
    gyro_fs_ = fs;
    gyro_lsb_per_dps_ = GYRO_SENS[idx];
    return ESP_OK;
}

esp_err_t Mpu6050::setSampleRate(uint16_t hz)
{
    ESP_RETURN_ON_FALSE(i2c_ && hz > 0, ESP_ERR_INVALID_ARG, TAG, "bad args");
    const uint32_t base = (dlpf_ == Dlpf::Hz260) ? 8000 : 1000;
    ESP_RETURN_ON_FALSE(hz <= base, ESP_ERR_INVALID_ARG, TAG, "rate %u > %u Hz",
                        static_cast<unsigned>(hz), static_cast<unsigned>(base));
    const uint32_t div = base / hz - 1;
    ESP_RETURN_ON_FALSE(div <= 255, ESP_ERR_INVALID_ARG, TAG, "rate %u Hz too low", static_cast<unsigned>(hz));
    ESP_RETURN_ON_ERROR(writeReg(REG_SMPLRT_DIV, static_cast<uint8_t>(div)), TAG, "smplrt_div");
    sample_rate_hz_ = hz;
    return ESP_OK;
}

esp_err_t Mpu6050::setDlpf(Dlpf dlpf)
{
    const uint8_t idx = static_cast<uint8_t>(dlpf);
    ESP_RETURN_ON_FALSE(i2c_ && idx <= static_cast<uint8_t>(Dlpf::Hz5), ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_ERROR(updateReg(REG_CONFIG, 0x07, idx), TAG, "dlpf");
    const bool base_changed = (dlpf_ == Dlpf::Hz260) != (dlpf == Dlpf::Hz260);
    dlpf_ = dlpf;
    /* The base rate changes between 8 kHz and 1 kHz, so recompute the divider */
    if (base_changed && sample_rate_hz_) {
        return setSampleRate(sample_rate_hz_);
    }
    return ESP_OK;
}

esp_err_t Mpu6050::sleep(bool enable)
{
    ESP_RETURN_ON_FALSE(i2c_, ESP_ERR_INVALID_ARG, TAG, "bad args");
    return updateReg(REG_PWR_MGMT_1, PWR1_SLEEP, enable ? PWR1_SLEEP : 0);
}

/* ---------- data ---------- */

esp_err_t Mpu6050::readRaw(Raw &raw)
{
    /* Layout: ACCEL_X/Y/Z (6), TEMP (2), GYRO_X/Y/Z (6). One burst keeps the samples coherent. */
    uint8_t b[14];
    ESP_RETURN_ON_ERROR(readRegs(REG_ACCEL_XOUT_H, b, sizeof(b)), TAG, "burst read");

    raw.ax   = be16(&b[0]);
    raw.ay   = be16(&b[2]);
    raw.az   = be16(&b[4]);
    raw.temp = be16(&b[6]);
    raw.gx   = be16(&b[8]);
    raw.gy   = be16(&b[10]);
    raw.gz   = be16(&b[12]);
    return ESP_OK;
}

esp_err_t Mpu6050::read(Data &d)
{
    Raw r;
    ESP_RETURN_ON_ERROR(readRaw(r), TAG, "read raw");

    d.ax = r.ax / accel_lsb_per_g_;
    d.ay = r.ay / accel_lsb_per_g_;
    d.az = r.az / accel_lsb_per_g_;
    d.gx = r.gx / gyro_lsb_per_dps_ - gyro_bias_[0];
    d.gy = r.gy / gyro_lsb_per_dps_ - gyro_bias_[1];
    d.gz = r.gz / gyro_lsb_per_dps_ - gyro_bias_[2];
    d.temp_c = r.temp / 340.0f + 36.53f;   /* datasheet formula */
    return ESP_OK;
}

esp_err_t Mpu6050::calibrateGyro(uint16_t samples)
{
    ESP_RETURN_ON_FALSE(i2c_ && samples, ESP_ERR_INVALID_ARG, TAG, "bad args");

    int64_t sum[3] = {0, 0, 0};
    const TickType_t period = pdMS_TO_TICKS(1000 / (sample_rate_hz_ ? sample_rate_hz_ : 100)) + 1;

    for (uint16_t i = 0; i < samples; i++) {
        Raw r;
        ESP_RETURN_ON_ERROR(readRaw(r), TAG, "read raw");
        sum[0] += r.gx;
        sum[1] += r.gy;
        sum[2] += r.gz;
        vTaskDelay(period);
    }
    for (int i = 0; i < 3; i++) {
        gyro_bias_[i] = (static_cast<float>(sum[i]) / samples) / gyro_lsb_per_dps_;
    }
    ESP_LOGI(TAG, "gyro bias: %.3f %.3f %.3f dps", gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
    return ESP_OK;
}

esp_err_t Mpu6050::calibrateLevel(uint16_t samples)
{
    ESP_RETURN_ON_FALSE(i2c_ && samples, ESP_ERR_INVALID_ARG, TAG, "bad args");

    float sx = 0, sy = 0, sz = 0;
    const TickType_t period = pdMS_TO_TICKS(1000 / (sample_rate_hz_ ? sample_rate_hz_ : 100)) + 1;

    for (uint16_t i = 0; i < samples; i++) {
        Data d;
        ESP_RETURN_ON_ERROR(read(d), TAG, "read");
        sx += d.ax; sy += d.ay; sz += d.az;
        vTaskDelay(period);
    }
    const float ax = sx / samples, ay = sy / samples, az = sz / samples;

    roll_off_  = std::atan2(ay, az) * RAD2DEG;
    pitch_off_ = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * RAD2DEG;

    resetAngles();   /* drop stale filter state */
    return ESP_OK;
}

/* ---------- angle estimate ---------- */

esp_err_t Mpu6050::readAngles(ComplementaryFilter &cf, Data *d)
{
    ESP_RETURN_ON_FALSE(i2c_, ESP_ERR_INVALID_ARG, TAG, "bad args");

    Data tmp;
    Data &out = d ? *d : tmp;
    ESP_RETURN_ON_ERROR(read(out), TAG, "read");

    const int64_t now = esp_timer_get_time();
    const float dt = cf.last_us() ? (now - cf.last_us()) / 1e6f : 0.0f;
    cf.set_last_us(now);

    cf.update(out, dt);
    return ESP_OK;
}

esp_err_t Mpu6050::getAngles(EulerAngles &angles)
{
    ESP_RETURN_ON_ERROR(readAngles(cf_, nullptr), TAG, "update angles");
    angles.roll  = cf_.roll()  - roll_off_;
    angles.pitch = cf_.pitch() - pitch_off_;
    angles.yaw = 0.0f;
    return ESP_OK;
}

esp_err_t Mpu6050::setFilterAlpha(float alpha)
{
    ESP_RETURN_ON_FALSE(alpha >= 0.0f && alpha <= 1.0f, ESP_ERR_INVALID_ARG, TAG, "bad args");
    cf_.setAlpha(alpha);
    return ESP_OK;
}