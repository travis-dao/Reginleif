/*
 * PCA9685 16-channel, 12-bit PWM driver for ESP-IDF (i2c_master API, v5.2+)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9685_I2C_ADDR_DEFAULT  0x40
#define PCA9685_NUM_CHANNELS      16
#define PCA9685_CHANNEL_ALL       16      /* pseudo-channel: ALL_LED registers */
#define PCA9685_MAX_COUNT         4096    /* 12-bit counter; also "fully on"   */
#define PCA9685_OSC_HZ_DEFAULT    25000000
#define PCA9685_FREQ_MIN_HZ       24.0f
#define PCA9685_FREQ_MAX_HZ       1526.0f

typedef struct pca9685_t *pca9685_handle_t;

typedef struct {
    i2c_master_bus_handle_t bus;   /**< Existing I2C master bus (required)            */
    uint8_t  addr;                 /**< 7-bit address, 0 -> 0x40                      */
    uint32_t scl_speed_hz;         /**< 0 -> 400 kHz (chip supports up to 1 MHz)      */
    uint32_t osc_clock_hz;         /**< 0 -> 25 MHz. Tune to calibrate output freq.   */
    float    pwm_freq_hz;          /**< 24..1526 Hz, 0 -> 200 Hz                      */
    bool     invert_output;        /**< MODE2.INVRT                                   */
    bool     open_drain;           /**< false -> totem-pole (OUTDRV), true -> open-drain */
    bool     stagger_outputs;      /**< Spread channel ON edges to reduce current spikes */
    uint32_t timeout_ms;           /**< I2C transaction timeout, 0 -> 100 ms          */
} pca9685_config_t;

/**
 * @brief Probe the chip, configure it, set PWM frequency, all outputs off, wake it.
 */
esp_err_t pca9685_new(const pca9685_config_t *config, pca9685_handle_t *out_handle);

/**
 * @brief Free driver resources. Outputs are left as they are (set duty 0 on
 *        PCA9685_CHANNEL_ALL first if you want them off).
 */
esp_err_t pca9685_del(pca9685_handle_t handle);

/** Set PWM frequency (puts chip briefly to sleep; outputs glitch). */
esp_err_t pca9685_set_pwm_freq(pca9685_handle_t handle, float freq_hz);

/** Actual frequency after prescaler rounding. */
esp_err_t pca9685_get_pwm_freq(pca9685_handle_t handle, float *out_freq_hz);

/**
 * @brief Raw register-level control.
 * @param channel 0..15 or PCA9685_CHANNEL_ALL
 * @param on      count (0..4095) at which output goes high; 4096 = fully on
 * @param off     count (0..4095) at which output goes low;  4096 = fully off
 *                (fully-off wins if both are 4096)
 */
esp_err_t pca9685_set_pwm(pca9685_handle_t handle, uint8_t channel,
                          uint16_t on, uint16_t off);

/** Duty in counts: 0 = fully off, 4096 = fully on. */
esp_err_t pca9685_set_duty(pca9685_handle_t handle, uint8_t channel, uint16_t duty);

/** Duty in percent, 0.0 .. 100.0. */
esp_err_t pca9685_set_duty_percent(pca9685_handle_t handle, uint8_t channel, float percent);

/** Pulse width in microseconds at the current PWM frequency (handy for servos). */
esp_err_t pca9685_set_pulse_us(pca9685_handle_t handle, uint8_t channel, float pulse_us);

/** Enter low-power mode (oscillator off, outputs stop). */
esp_err_t pca9685_sleep(pca9685_handle_t handle);

/** Leave low-power mode and restart PWM with previous register contents. */
esp_err_t pca9685_wake(pca9685_handle_t handle);

/**
 * @brief I2C general-call software reset (0x00 / 0x06).
 *        Resets EVERY PCA9685 on the bus. Call before pca9685_new() if desired.
 */
esp_err_t pca9685_general_call_reset(i2c_master_bus_handle_t bus);



#ifdef __cplusplus
}
#endif