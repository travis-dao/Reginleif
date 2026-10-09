/*
 * MPU6050 driver for ESP-IDF (>= v5.2) using the new i2c_master API.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 7-bit I2C addresses (selected by the AD0 pin) */
#define MPU6050_ADDR_AD0_LOW   0x68
#define MPU6050_ADDR_AD0_HIGH  0x69

typedef enum {
    MPU6050_ACCEL_FS_2G  = 0,
    MPU6050_ACCEL_FS_4G  = 1,
    MPU6050_ACCEL_FS_8G  = 2,
    MPU6050_ACCEL_FS_16G = 3,
} mpu6050_accel_fs_t;

typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0,
    MPU6050_GYRO_FS_500DPS  = 1,
    MPU6050_GYRO_FS_1000DPS = 2,
    MPU6050_GYRO_FS_2000DPS = 3,
} mpu6050_gyro_fs_t;

/* Digital low-pass filter setting (CONFIG register DLPF_CFG).
 * DLPF_256HZ disables the filter and raises the gyro output rate to 8 kHz. */
typedef enum {
    MPU6050_DLPF_260HZ = 0,   /* accel 260 Hz / gyro 256 Hz, gyro rate 8 kHz */
    MPU6050_DLPF_184HZ = 1,   /* accel 184 Hz / gyro 188 Hz, gyro rate 1 kHz */
    MPU6050_DLPF_94HZ  = 2,
    MPU6050_DLPF_44HZ  = 3,
    MPU6050_DLPF_21HZ  = 4,
    MPU6050_DLPF_10HZ  = 5,
    MPU6050_DLPF_5HZ   = 6,
} mpu6050_dlpf_t;

typedef struct {
    i2c_master_bus_handle_t bus;   /* An already-created I2C master bus */
    uint8_t  addr;                 /* MPU6050_ADDR_AD0_LOW or _HIGH */
    uint32_t scl_speed_hz;         /* Up to 400000 */
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t  gyro_fs;
    mpu6050_dlpf_t     dlpf;
    uint16_t sample_rate_hz;       /* Internal sample rate, see mpu6050_set_sample_rate() */
    float    filter_alpha;         /* Complementary filter gyro weight for mpu6050_get_angles(); 0 = default (0.98) */
} mpu6050_config_t;

/* Fill in everything except .bus */
#define MPU6050_CONFIG_DEFAULT() {            \
    .bus            = NULL,                   \
    .addr           = MPU6050_ADDR_AD0_LOW,   \
    .scl_speed_hz   = 400000,                 \
    .accel_fs       = MPU6050_ACCEL_FS_4G,    \
    .gyro_fs        = MPU6050_GYRO_FS_500DPS, \
    .dlpf           = MPU6050_DLPF_44HZ,      \
    .sample_rate_hz = 200,                    \
    .filter_alpha   = 0.98f,                  \
}

typedef struct {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
} mpu6050_raw_t;

typedef struct {
    float ax, ay, az;      /* g */
    float gx, gy, gz;      /* deg/s (gyro bias already removed) */
    float temp_c;          /* degrees Celsius */
} mpu6050_data_t;

typedef struct mpu6050_dev *mpu6050_handle_t;

/**
 * @brief Create the device, verify WHO_AM_I, reset, and apply the configuration.
 */
esp_err_t mpu6050_init(const mpu6050_config_t *cfg, mpu6050_handle_t *out_handle);

/**
 * @brief Put the chip to sleep and release the I2C device (the bus is NOT deleted).
 */
esp_err_t mpu6050_deinit(mpu6050_handle_t dev);

/** @brief Read the 14 data bytes in one burst and return raw sensor values. */
esp_err_t mpu6050_read_raw(mpu6050_handle_t dev, mpu6050_raw_t *raw);

/** @brief Read and convert to physical units. */
esp_err_t mpu6050_read(mpu6050_handle_t dev, mpu6050_data_t *data);

esp_err_t mpu6050_set_accel_fs(mpu6050_handle_t dev, mpu6050_accel_fs_t fs);
esp_err_t mpu6050_set_gyro_fs(mpu6050_handle_t dev, mpu6050_gyro_fs_t fs);
esp_err_t mpu6050_set_dlpf(mpu6050_handle_t dev, mpu6050_dlpf_t dlpf);

/**
 * @brief Set the sample rate. Rate = gyro_output_rate / (1 + SMPLRT_DIV),
 *        where gyro_output_rate is 8 kHz for DLPF_260HZ and 1 kHz otherwise.
 *        Valid range: 4..1000 Hz (32..8000 Hz with the filter disabled).
 */
esp_err_t mpu6050_set_sample_rate(mpu6050_handle_t dev, uint16_t hz);

/** @brief Enter or leave low-power sleep mode. */
esp_err_t mpu6050_sleep(mpu6050_handle_t dev, bool enable);

/**
 * @brief Estimate gyro bias by averaging @p samples readings. Keep the board still.
 *        The bias is then subtracted in mpu6050_read().
 */
esp_err_t mpu6050_calibrate_gyro(mpu6050_handle_t dev, uint16_t samples);
esp_err_t mpu6050_calibrate_level(mpu6050_handle_t dev, uint16_t samples);

/* ---------- Complementary filter (roll / pitch) ---------- */

typedef struct {
    float alpha;        /* Gyro weight, 0..1 (typ. 0.95..0.99). Higher = smoother, slower drift correction */
    float roll;         /* degrees, rotation about X */
    float pitch;        /* degrees, rotation about Y */
    int64_t last_us;    /* timestamp of the last mpu6050_read_angles() call */
    bool seeded;        /* false until the first sample has initialised the angles */
} mpu6050_cf_t;

/**
 * @brief Initialise the filter state.
 * @param alpha Gyro weight. Related to a time constant tau by alpha = tau / (tau + dt).
 *              e.g. 0.98 at 200 Hz is tau of about 0.25 s.
 */
void mpu6050_cf_init(mpu6050_cf_t *cf, float alpha);

/**
 * @brief Run one filter step on an already-read sample (pure math, no I2C).
 *        Use this if you manage timing yourself, e.g. from a timer or a data-ready ISR.
 *        The first call seeds the angles from the accelerometer alone.
 *
 * @param cf  Filter state
 * @param d   Scaled sample from mpu6050_read()
 * @param dt  Seconds since the previous update
 */
void mpu6050_cf_update(mpu6050_cf_t *cf, const mpu6050_data_t *d, float dt);

/**
 * @brief Read the sensor, measure dt with esp_timer, and update the filter.
 *
 * @param dev Device handle
 * @param cf  Filter state
 * @param d   Optional: receives the scaled sample (may be NULL)
 */
esp_err_t mpu6050_read_angles(mpu6050_handle_t dev, mpu6050_cf_t *cf, mpu6050_data_t *d);

/* ---------- Built-in angle estimate ---------- */

typedef struct {
    float roll;     /* degrees, rotation about X */
    float pitch;    /* degrees, rotation about Y */
} mpu6050_angles_t;

/**
 * @brief Read the sensor and return the filtered roll and pitch.
 *
 * The complementary filter state lives inside the device handle, so there is
 * nothing to set up besides mpu6050_init(). Call this at a steady rate
 * (ideally matching the configured sample rate). Not thread-safe per handle.
 */
esp_err_t mpu6050_get_angles(mpu6050_handle_t dev, mpu6050_angles_t *angles);

/** @brief Change the filter's gyro weight (0..1) at runtime. */
esp_err_t mpu6050_set_filter_alpha(mpu6050_handle_t dev, float alpha);

/** @brief Discard the current estimate; the next get_angles() reseeds from the accelerometer. */
esp_err_t mpu6050_reset_angles(mpu6050_handle_t dev);

/* Low-level register access */
esp_err_t mpu6050_read_regs(mpu6050_handle_t dev, uint8_t reg, uint8_t *buf, size_t len);
esp_err_t mpu6050_write_reg(mpu6050_handle_t dev, uint8_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif