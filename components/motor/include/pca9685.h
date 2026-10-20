/*
 * PCA9685 16-channel, 12-bit PWM driver for ESP-IDF (i2c_master API, v5.2+)
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

constexpr uint8_t  I2C_ADDR_DEFAULT = 0x40;
constexpr uint8_t  NUM_CHANNELS     = 16;
constexpr uint8_t  CHANNEL_ALL      = 16;        /* pseudo-channel: ALL_LED registers */
constexpr uint16_t MAX_COUNT        = 4096;      /* 12-bit counter; also "fully on"   */
constexpr uint32_t OSC_HZ_DEFAULT   = 25000000;
constexpr float    FREQ_MIN_HZ      = 24.0f;
constexpr float    FREQ_MAX_HZ      = 1526.0f;

class Pca9685 {
    public:
        struct Config {
            i2c_master_bus_handle_t bus = nullptr;  /**< Existing I2C master bus (required)            */
            uint8_t  addr            = 0;           /**< 7-bit address, 0 -> 0x40                      */
            uint32_t scl_speed_hz    = 0;           /**< 0 -> 400 kHz (chip supports up to 1 MHz)      */
            uint32_t osc_clock_hz    = 0;           /**< 0 -> 25 MHz. Tune to calibrate output freq.   */
            float    pwm_freq_hz     = 0.0f;        /**< 24..1526 Hz, 0 -> 200 Hz                      */
            bool     invert_output   = false;       /**< MODE2.INVRT                                   */
            bool     open_drain      = false;       /**< false -> totem-pole (OUTDRV), true -> open-drain */
            bool     stagger_outputs = false;       /**< Spread channel ON edges to reduce current spikes */
            uint32_t timeout_ms      = 0;           /**< I2C transaction timeout, 0 -> 100 ms          */
        };
        
        Pca9685() = default;
        ~Pca9685();

        Pca9685(const Pca9685 &)            = delete;
        Pca9685 &operator=(const Pca9685 &) = delete;

        /** Probe the chip, configure it, set PWM frequency, all outputs off, wake it. */
        esp_err_t init(const Config &cfg);

        /**
        * Free driver resources. Outputs are left as they are (set duty 0 on
        * CHANNEL_ALL first if you want them off).
        */
        esp_err_t deinit();

        /** Set PWM frequency (puts chip briefly to sleep; outputs glitch). */
        esp_err_t setPwmFreq(float freq_hz);

        /** Actual frequency after prescaler rounding. */
        esp_err_t getPwmFreq(float &out_freq_hz);

        /**
        * Raw register-level control.
        * @param ch  0..15 or CHANNEL_ALL
        * @param on  count (0..4095) at which output goes high; 4096 = fully on
        * @param off count (0..4095) at which output goes low;  4096 = fully off
        *            (fully-off wins if both are 4096)
        */
        esp_err_t setPwm(uint8_t ch, uint16_t on, uint16_t off);

        /** Duty in counts: 0 = fully off, 4096 = fully on. */
        esp_err_t setDuty(uint8_t ch, uint16_t duty);

        /** Duty in percent, 0.0 .. 100.0. */
        esp_err_t setDutyPercent(uint8_t ch, float percent);

        /** Pulse width in microseconds at the current PWM frequency (handy for servos). */
        esp_err_t setPulseUs(uint8_t ch, float pulse_us);

        /** Enter low-power mode (oscillator off, outputs stop). */
        esp_err_t sleep();

        /** Leave low-power mode and restart PWM with previous register contents. */
        esp_err_t wake();

        bool initialized() const { return dev_ != nullptr; }

        /**
        * I2C general-call software reset (0x00 / 0x06).
        * Resets EVERY PCA9685 on the bus. Call before init() if desired.
        */
        static esp_err_t generalCallReset(i2c_master_bus_handle_t bus);

        // burst write
        void computeOnOff(uint8_t ch, uint16_t duty, uint16_t &on, uint16_t &off) const;
        esp_err_t setDuties(uint8_t first_ch, const uint16_t *duty, size_t count);


    private:
        esp_err_t configure(const Config &cfg);
        void      release();

        esp_err_t regWrite(uint8_t reg, const uint8_t *data, size_t len);
        esp_err_t regWrite8(uint8_t reg, uint8_t val);
        esp_err_t regRead8(uint8_t reg, uint8_t *val);

        /* Callers of these hold the lock */
        esp_err_t doSleep();
        esp_err_t doWake();
        esp_err_t doSetFreq(float freq_hz);
        esp_err_t writeChannel(uint8_t ch, uint16_t on, uint16_t off);
        esp_err_t writeDuty(uint8_t ch, uint16_t duty);

        i2c_master_dev_handle_t dev_        = nullptr;
        SemaphoreHandle_t       lock_       = nullptr;
        uint32_t                osc_hz_     = OSC_HZ_DEFAULT;
        float                   freq_hz_    = 0.0f;
        int                     timeout_ms_ = 100;
        bool                    stagger_    = false;
};