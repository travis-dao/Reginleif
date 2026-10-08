#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"

static const char *TAG = "mpu6050";

/* Registers */
#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75

/* PWR_MGMT_1 bits */
#define PWR1_DEVICE_RESET  (1 << 7)
#define PWR1_SLEEP         (1 << 6)
#define PWR1_CLKSEL_PLL_X  0x01   /* PLL with X-axis gyro reference: more stable than the internal RC */

#define WHO_AM_I_VALUE     0x68
#define I2C_TIMEOUT_MS     100

#define RAD2DEG            57.2957795f
#define CF_MAX_DT_S        0.5f    /* a longer gap means a stall: reseed instead of integrating */
#define CF_ACCEL_MIN_G     0.5f    /* outside this window the accel is dominated by motion, */
#define CF_ACCEL_MAX_G     1.5f    /* so it is a poor gravity reference and is ignored      */

struct mpu6050_dev {
    i2c_master_dev_handle_t i2c;
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t  gyro_fs;
    mpu6050_dlpf_t     dlpf;
    uint16_t sample_rate_hz;
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;
    float gyro_bias[3];   /* deg/s */
    mpu6050_cf_t cf;      /* filter state for mpu6050_get_angles() */
};

static const float ACCEL_SENS[] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float GYRO_SENS[]  = { 131.0f, 65.5f, 32.8f, 16.4f };

/* ---------- low-level access ---------- */

esp_err_t mpu6050_read_regs(mpu6050_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(dev && buf && len, ESP_ERR_INVALID_ARG, TAG, "bad args");
    return i2c_master_transmit_receive(dev->i2c, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

esp_err_t mpu6050_write_reg(mpu6050_handle_t dev, uint8_t reg, uint8_t val)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "bad args");
    const uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->i2c, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

/* Read-modify-write helper */
static esp_err_t update_reg(mpu6050_handle_t dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t cur;
    ESP_RETURN_ON_ERROR(mpu6050_read_regs(dev, reg, &cur, 1), TAG, "read reg 0x%02X", reg);
    cur = (cur & ~mask) | (val & mask);
    return mpu6050_write_reg(dev, reg, cur);
}

/* ---------- configuration ---------- */

esp_err_t mpu6050_set_accel_fs(mpu6050_handle_t dev, mpu6050_accel_fs_t fs)
{
    ESP_RETURN_ON_FALSE(dev && fs <= MPU6050_ACCEL_FS_16G, ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_ERROR(mpu6050_write_reg(dev, REG_ACCEL_CONFIG, (uint8_t)(fs << 3)), TAG, "accel cfg");
    dev->accel_fs = fs;
    dev->accel_lsb_per_g = ACCEL_SENS[fs];
    return ESP_OK;
}

esp_err_t mpu6050_set_gyro_fs(mpu6050_handle_t dev, mpu6050_gyro_fs_t fs)
{
    ESP_RETURN_ON_FALSE(dev && fs <= MPU6050_GYRO_FS_2000DPS, ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_ERROR(mpu6050_write_reg(dev, REG_GYRO_CONFIG, (uint8_t)(fs << 3)), TAG, "gyro cfg");
    dev->gyro_fs = fs;
    dev->gyro_lsb_per_dps = GYRO_SENS[fs];
    return ESP_OK;
}

esp_err_t mpu6050_set_sample_rate(mpu6050_handle_t dev, uint16_t hz)
{
    ESP_RETURN_ON_FALSE(dev && hz > 0, ESP_ERR_INVALID_ARG, TAG, "bad args");
    const uint32_t base = (dev->dlpf == MPU6050_DLPF_260HZ) ? 8000 : 1000;
    ESP_RETURN_ON_FALSE(hz <= base, ESP_ERR_INVALID_ARG, TAG, "rate %u > %u Hz", hz, (unsigned)base);
    const uint32_t div = base / hz - 1;
    ESP_RETURN_ON_FALSE(div <= 255, ESP_ERR_INVALID_ARG, TAG, "rate %u Hz too low", hz);
    ESP_RETURN_ON_ERROR(mpu6050_write_reg(dev, REG_SMPLRT_DIV, (uint8_t)div), TAG, "smplrt_div");
    dev->sample_rate_hz = hz;
    return ESP_OK;
}

esp_err_t mpu6050_set_dlpf(mpu6050_handle_t dev, mpu6050_dlpf_t dlpf)
{
    ESP_RETURN_ON_FALSE(dev && dlpf <= MPU6050_DLPF_5HZ, ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_ERROR(update_reg(dev, REG_CONFIG, 0x07, (uint8_t)dlpf), TAG, "dlpf");
    const bool base_changed = (dev->dlpf == MPU6050_DLPF_260HZ) != (dlpf == MPU6050_DLPF_260HZ);
    dev->dlpf = dlpf;
    /* The base rate changes between 8 kHz and 1 kHz, so recompute the divider */
    if (base_changed && dev->sample_rate_hz) {
        return mpu6050_set_sample_rate(dev, dev->sample_rate_hz);
    }
    return ESP_OK;
}

esp_err_t mpu6050_sleep(mpu6050_handle_t dev, bool enable)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "bad args");
    return update_reg(dev, REG_PWR_MGMT_1, PWR1_SLEEP, enable ? PWR1_SLEEP : 0);
}

/* ---------- lifecycle ---------- */

esp_err_t mpu6050_init(const mpu6050_config_t *cfg, mpu6050_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->bus && out_handle, ESP_ERR_INVALID_ARG, TAG, "bad args");

    mpu6050_handle_t dev = calloc(1, sizeof(*dev));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no mem");

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->addr,
        .scl_speed_hz    = cfg->scl_speed_hz ? cfg->scl_speed_hz : 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(cfg->bus, &dev_cfg, &dev->i2c);
    ESP_GOTO_ON_ERROR(ret, fail_free, TAG, "add device failed");

    /* Identify the chip */
    uint8_t who = 0;
    ret = mpu6050_read_regs(dev, REG_WHO_AM_I, &who, 1);
    ESP_GOTO_ON_ERROR(ret, fail_rm, TAG, "WHO_AM_I read failed (check wiring/address)");
    if (who != WHO_AM_I_VALUE) {
        /* Some clones report 0x70/0x72 etc.; relax this check if you use one. */
        ESP_LOGE(TAG, "unexpected WHO_AM_I 0x%02X (expected 0x%02X)", who, WHO_AM_I_VALUE);
        ret = ESP_ERR_NOT_FOUND;
        goto fail_rm;
    }

    /* Reset, then wake up with the gyro PLL as clock source */
    ret = mpu6050_write_reg(dev, REG_PWR_MGMT_1, PWR1_DEVICE_RESET);
    ESP_GOTO_ON_ERROR(ret, fail_rm, TAG, "reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = mpu6050_write_reg(dev, REG_PWR_MGMT_1, PWR1_CLKSEL_PLL_X);
    ESP_GOTO_ON_ERROR(ret, fail_rm, TAG, "wake failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Apply configuration (DLPF first: the sample rate base depends on it) */
    dev->dlpf = MPU6050_DLPF_260HZ;   /* reset value, keeps set_dlpf() bookkeeping right */
    ret = mpu6050_set_dlpf(dev, cfg->dlpf);
    ESP_GOTO_ON_ERROR(ret, fail_rm, TAG, "dlpf");
    ret = mpu6050_set_accel_fs(dev, cfg->accel_fs);
    ESP_GOTO_ON_ERROR(ret, fail_rm, TAG, "accel fs");
    ret = mpu6050_set_gyro_fs(dev, cfg->gyro_fs);
    ESP_GOTO_ON_ERROR(ret, fail_rm, TAG, "gyro fs");
    ret = mpu6050_set_sample_rate(dev, cfg->sample_rate_hz ? cfg->sample_rate_hz : 100);
    ESP_GOTO_ON_ERROR(ret, fail_rm, TAG, "sample rate");

    mpu6050_cf_init(&dev->cf, cfg->filter_alpha > 0.0f ? cfg->filter_alpha : 0.98f);

    ESP_LOGI(TAG, "MPU6050 ready @0x%02X", cfg->addr);
    *out_handle = dev;
    return ESP_OK;

fail_rm:
    i2c_master_bus_rm_device(dev->i2c);
fail_free:
    free(dev);
    return ret;
}

esp_err_t mpu6050_deinit(mpu6050_handle_t dev)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "bad args");
    mpu6050_sleep(dev, true);   /* best effort */
    esp_err_t ret = i2c_master_bus_rm_device(dev->i2c);
    free(dev);
    return ret;
}

/* ---------- data ---------- */

static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

esp_err_t mpu6050_read_raw(mpu6050_handle_t dev, mpu6050_raw_t *raw)
{
    ESP_RETURN_ON_FALSE(dev && raw, ESP_ERR_INVALID_ARG, TAG, "bad args");

    /* Layout: ACCEL_X/Y/Z (6), TEMP (2), GYRO_X/Y/Z (6). One burst keeps the samples coherent. */
    uint8_t b[14];
    ESP_RETURN_ON_ERROR(mpu6050_read_regs(dev, REG_ACCEL_XOUT_H, b, sizeof(b)), TAG, "burst read");

    raw->ax   = be16(&b[0]);
    raw->ay   = be16(&b[2]);
    raw->az   = be16(&b[4]);
    raw->temp = be16(&b[6]);
    raw->gx   = be16(&b[8]);
    raw->gy   = be16(&b[10]);
    raw->gz   = be16(&b[12]);
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_handle_t dev, mpu6050_data_t *d)
{
    ESP_RETURN_ON_FALSE(dev && d, ESP_ERR_INVALID_ARG, TAG, "bad args");

    mpu6050_raw_t r;
    ESP_RETURN_ON_ERROR(mpu6050_read_raw(dev, &r), TAG, "read raw");

    d->ax = r.ax / dev->accel_lsb_per_g;
    d->ay = r.ay / dev->accel_lsb_per_g;
    d->az = r.az / dev->accel_lsb_per_g;
    d->gx = r.gx / dev->gyro_lsb_per_dps - dev->gyro_bias[0];
    d->gy = r.gy / dev->gyro_lsb_per_dps - dev->gyro_bias[1];
    d->gz = r.gz / dev->gyro_lsb_per_dps - dev->gyro_bias[2];
    d->temp_c = r.temp / 340.0f + 36.53f;   /* datasheet formula */
    return ESP_OK;
}

esp_err_t mpu6050_calibrate_gyro(mpu6050_handle_t dev, uint16_t samples)
{
    ESP_RETURN_ON_FALSE(dev && samples, ESP_ERR_INVALID_ARG, TAG, "bad args");

    int64_t sum[3] = {0};
    const TickType_t period = pdMS_TO_TICKS(1000 / (dev->sample_rate_hz ? dev->sample_rate_hz : 100)) + 1;

    for (uint16_t i = 0; i < samples; i++) {
        mpu6050_raw_t r;
        ESP_RETURN_ON_ERROR(mpu6050_read_raw(dev, &r), TAG, "read raw");
        sum[0] += r.gx;
        sum[1] += r.gy;
        sum[2] += r.gz;
        vTaskDelay(period);
    }
    for (int i = 0; i < 3; i++) {
        dev->gyro_bias[i] = ((float)sum[i] / samples) / dev->gyro_lsb_per_dps;
    }
    ESP_LOGI(TAG, "gyro bias: %.3f %.3f %.3f dps", dev->gyro_bias[0], dev->gyro_bias[1], dev->gyro_bias[2]);
    return ESP_OK;
}

/* ---------- complementary filter ---------- */

void mpu6050_cf_init(mpu6050_cf_t *cf, float alpha)
{
    if (!cf) {
        return;
    }
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    memset(cf, 0, sizeof(*cf));
    cf->alpha = alpha;
}

void mpu6050_cf_update(mpu6050_cf_t *cf, const mpu6050_data_t *d, float dt)
{
    if (!cf || !d) {
        return;
    }

    /* Angles implied by gravity alone */
    const float norm = sqrtf(d->ax * d->ax + d->ay * d->ay + d->az * d->az);
    const bool accel_ok = (norm > CF_ACCEL_MIN_G) && (norm < CF_ACCEL_MAX_G);
    float acc_roll = 0, acc_pitch = 0;
    if (accel_ok) {
        acc_roll  = atan2f(d->ay, d->az) * RAD2DEG;
        acc_pitch = atan2f(-d->ax, sqrtf(d->ay * d->ay + d->az * d->az)) * RAD2DEG;
    }

    /* First sample, or a long gap: start from the accelerometer */
    if (!cf->seeded || dt <= 0.0f || dt > CF_MAX_DT_S) {
        if (accel_ok) {
            cf->roll   = acc_roll;
            cf->pitch  = acc_pitch;
            cf->seeded = true;
        }
        return;
    }

    /* Integrate the gyro (small-angle approximation: body rates used as Euler rates) */
    const float gyro_roll  = cf->roll  + d->gx * dt;
    const float gyro_pitch = cf->pitch + d->gy * dt;

    if (accel_ok) {
        cf->roll  = cf->alpha * gyro_roll  + (1.0f - cf->alpha) * acc_roll;
        cf->pitch = cf->alpha * gyro_pitch + (1.0f - cf->alpha) * acc_pitch;
    } else {
        cf->roll  = gyro_roll;     /* gyro only while the accel is unreliable */
        cf->pitch = gyro_pitch;
    }
}

esp_err_t mpu6050_read_angles(mpu6050_handle_t dev, mpu6050_cf_t *cf, mpu6050_data_t *d)
{
    ESP_RETURN_ON_FALSE(dev && cf, ESP_ERR_INVALID_ARG, TAG, "bad args");

    mpu6050_data_t tmp;
    mpu6050_data_t *out = d ? d : &tmp;
    ESP_RETURN_ON_ERROR(mpu6050_read(dev, out), TAG, "read");

    const int64_t now = esp_timer_get_time();
    const float dt = cf->last_us ? (now - cf->last_us) / 1e6f : 0.0f;
    cf->last_us = now;

    mpu6050_cf_update(cf, out, dt);
    return ESP_OK;
}

/* ---------- built-in angle estimate ---------- */

esp_err_t mpu6050_get_angles(mpu6050_handle_t dev, mpu6050_angles_t *angles)
{
    ESP_RETURN_ON_FALSE(dev && angles, ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_ERROR(mpu6050_read_angles(dev, &dev->cf, NULL), TAG, "update angles");
    angles->roll  = dev->cf.roll;
    angles->pitch = dev->cf.pitch;
    return ESP_OK;
}

esp_err_t mpu6050_set_filter_alpha(mpu6050_handle_t dev, float alpha)
{
    ESP_RETURN_ON_FALSE(dev && alpha >= 0.0f && alpha <= 1.0f, ESP_ERR_INVALID_ARG, TAG, "bad args");
    dev->cf.alpha = alpha;
    return ESP_OK;
}

esp_err_t mpu6050_reset_angles(mpu6050_handle_t dev)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "bad args");
    mpu6050_cf_init(&dev->cf, dev->cf.alpha);
    return ESP_OK;
}