#include <cmath>
#include <cstdlib>
#include <cstring>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "pca9685.h"

const char *const TAG = "pca9685";

/* Registers */
constexpr uint8_t REG_MODE1        = 0x00;
constexpr uint8_t REG_MODE2        = 0x01;
constexpr uint8_t REG_LED0_ON_L    = 0x06;
constexpr uint8_t REG_ALL_LED_ON_L = 0xFA;
constexpr uint8_t REG_PRESCALE     = 0xFE;

/* MODE1 bits */
constexpr uint8_t MODE1_RESTART = 0x80;
constexpr uint8_t MODE1_AI      = 0x20;   /* register auto-increment */
constexpr uint8_t MODE1_SLEEP   = 0x10;

/* MODE2 bits */
constexpr uint8_t MODE2_INVRT  = 0x10;
constexpr uint8_t MODE2_OUTDRV = 0x04;

constexpr int      PRESCALE_MIN       = 3;
constexpr int      PRESCALE_MAX       = 255;
constexpr float    DEFAULT_FREQ_HZ    = 200.0f;
constexpr uint32_t DEFAULT_TIMEOUT_MS = 100;

/* RAII lock: released on every return path */
class Guard {
    public:
        explicit Guard(SemaphoreHandle_t m) : m_(m) { xSemaphoreTake(m_, portMAX_DELAY); }
        ~Guard() { xSemaphoreGive(m_); }
        Guard(const Guard &)            = delete;
        Guard &operator=(const Guard &) = delete;
    private:
        SemaphoreHandle_t m_;
};

/* ---------- lifecycle ---------- */

Pca9685::~Pca9685()
{
    if (dev_) {
        deinit();
    }
}

esp_err_t Pca9685::init(const Config &cfg)
{
    ESP_RETURN_ON_FALSE(cfg.bus, ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_FALSE(!dev_, ESP_ERR_INVALID_STATE, TAG, "already initialized");

    const uint8_t  addr    = cfg.addr ? cfg.addr : I2C_ADDR_DEFAULT;
    const uint32_t timeout = cfg.timeout_ms ? cfg.timeout_ms : DEFAULT_TIMEOUT_MS;

    ESP_RETURN_ON_ERROR(i2c_master_probe(cfg.bus, addr, timeout), TAG,
                        "no device answering at 0x%02X", addr);

    osc_hz_     = cfg.osc_clock_hz ? cfg.osc_clock_hz : OSC_HZ_DEFAULT;
    timeout_ms_ = static_cast<int>(timeout);
    stagger_    = cfg.stagger_outputs;
    freq_hz_    = 0.0f;

    lock_ = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(lock_, ESP_ERR_NO_MEM, TAG, "no mem for mutex");

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = addr;
    dev_cfg.scl_speed_hz    = cfg.scl_speed_hz ? cfg.scl_speed_hz : 400000;
    esp_err_t ret = i2c_master_bus_add_device(cfg.bus, &dev_cfg, &dev_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add device failed");
        dev_ = nullptr;
        release();
        return ret;
    }

    ret = configure(cfg);
    if (ret != ESP_OK) {
        release();
        return ret;
    }
    return ESP_OK;
}

/* Everything after the I2C device is attached; the caller cleans up on failure. */
esp_err_t Pca9685::configure(const Config &cfg)
{
    const float freq = cfg.pwm_freq_hz > 0.0f ? cfg.pwm_freq_hz : DEFAULT_FREQ_HZ;

    /* Configure while asleep so PRE_SCALE can be written */
    const uint8_t mode2 = static_cast<uint8_t>((cfg.invert_output ? MODE2_INVRT : 0) |
                                               (cfg.open_drain    ? 0 : MODE2_OUTDRV));
    ESP_RETURN_ON_ERROR(regWrite8(REG_MODE2, mode2), TAG, "write MODE2");
    ESP_RETURN_ON_ERROR(regWrite8(REG_MODE1, MODE1_SLEEP | MODE1_AI), TAG, "write MODE1");

    ESP_RETURN_ON_ERROR(writeChannel(CHANNEL_ALL, 0, MAX_COUNT), TAG, "all off");
    ESP_RETURN_ON_ERROR(doSetFreq(freq), TAG, "set freq");
    ESP_RETURN_ON_ERROR(doWake(), TAG, "wake");
    return ESP_OK;
}

void Pca9685::release()
{
    if (dev_) {
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
    if (lock_) {
        vSemaphoreDelete(lock_);
        lock_ = nullptr;
    }
}

esp_err_t Pca9685::deinit()
{
    ESP_RETURN_ON_FALSE(dev_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    release();
    return ESP_OK;
}

/* ---------- low-level I2C helpers (caller holds the lock) ---------- */

esp_err_t Pca9685::regWrite(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[5];
    if (len > sizeof(buf) - 1) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = reg;
    std::memcpy(&buf[1], data, len);
    return i2c_master_transmit(dev_, buf, len + 1, timeout_ms_);
}

esp_err_t Pca9685::regWrite8(uint8_t reg, uint8_t val)
{
    return regWrite(reg, &val, 1);
}

esp_err_t Pca9685::regRead8(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev_, &reg, 1, val, 1, timeout_ms_);
}

/* ---------- power state ---------- */

esp_err_t Pca9685::doSleep()
{
    uint8_t m1;
    ESP_RETURN_ON_ERROR(regRead8(REG_MODE1, &m1), TAG, "read MODE1");
    if (m1 & MODE1_SLEEP) {
        return ESP_OK;
    }
    /* Never write RESTART=1 while going to sleep, or the restart state is lost */
    m1 = static_cast<uint8_t>((m1 & ~MODE1_RESTART) | MODE1_SLEEP);
    return regWrite8(REG_MODE1, m1);
}

esp_err_t Pca9685::doWake()
{
    uint8_t m1;
    ESP_RETURN_ON_ERROR(regRead8(REG_MODE1, &m1), TAG, "read MODE1");

    const bool restart = m1 & MODE1_RESTART;
    if (!(m1 & MODE1_SLEEP) && !restart) {
        return ESP_OK;
    }

    m1 &= static_cast<uint8_t>(~(MODE1_SLEEP | MODE1_RESTART));
    ESP_RETURN_ON_ERROR(regWrite8(REG_MODE1, m1), TAG, "clear SLEEP");
    esp_rom_delay_us(500);                 /* oscillator stabilisation (datasheet: 500 us) */
    if (restart) {
        ESP_RETURN_ON_ERROR(regWrite8(REG_MODE1, m1 | MODE1_RESTART), TAG, "set RESTART");
    }
    return ESP_OK;
}

/* ---------- frequency ---------- */

esp_err_t Pca9685::doSetFreq(float freq_hz)
{
    if (!(freq_hz >= FREQ_MIN_HZ && freq_hz <= FREQ_MAX_HZ)) {
        return ESP_ERR_INVALID_ARG;
    }
    const double p = std::round(static_cast<double>(osc_hz_) /
                                (4096.0 * static_cast<double>(freq_hz))) - 1.0;
    if (p < PRESCALE_MIN || p > PRESCALE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t prescale = static_cast<uint8_t>(p);

    uint8_t m1;
    ESP_RETURN_ON_ERROR(regRead8(REG_MODE1, &m1), TAG, "read MODE1");
    const bool was_asleep = m1 & MODE1_SLEEP;

    /* PRE_SCALE is only writable while SLEEP = 1 */
    ESP_RETURN_ON_ERROR(doSleep(), TAG, "sleep");
    ESP_RETURN_ON_ERROR(regWrite8(REG_PRESCALE, prescale), TAG, "write PRE_SCALE");
    if (!was_asleep) {
        ESP_RETURN_ON_ERROR(doWake(), TAG, "wake");
    }

    freq_hz_ = static_cast<float>(osc_hz_) / (4096.0f * static_cast<float>(prescale + 1));
    return ESP_OK;
}

/* ---------- channel write ---------- */

esp_err_t Pca9685::writeChannel(uint8_t ch, uint16_t on, uint16_t off)
{
    const uint8_t reg = (ch == CHANNEL_ALL) ? REG_ALL_LED_ON_L
                                            : static_cast<uint8_t>(REG_LED0_ON_L + 4 * ch);
    /* Bit 12 (0x1000) lands in bit 4 of the *_H register = full-on / full-off flag */
    const uint8_t d[4] = {
        static_cast<uint8_t>(on & 0xFF),  static_cast<uint8_t>((on >> 8) & 0x1F),
        static_cast<uint8_t>(off & 0xFF), static_cast<uint8_t>((off >> 8) & 0x1F),
    };
    return regWrite(reg, d, sizeof(d));
}

// esp_err_t Pca9685::writeDuty(uint8_t ch, uint16_t duty)
// {
//     uint16_t on, off;
//     if (duty == 0) {
//         on  = 0;
//         off = MAX_COUNT;                       /* full off */
//     } else if (duty >= MAX_COUNT) {
//         on  = MAX_COUNT;                       /* full on  */
//         off = 0;
//     } else {
//         on  = (stagger_ && ch < NUM_CHANNELS) ? static_cast<uint16_t>(ch * 256) : 0;
//         off = static_cast<uint16_t>((on + duty) & 0x0FFF);
//     }
//     return writeChannel(ch, on, off);
// }

void Pca9685::computeOnOff(uint8_t ch, uint16_t duty, uint16_t &on, uint16_t &off) const
{
    if (duty == 0) {
        on = 0;         off = MAX_COUNT;              /* full off */
    } else if (duty >= MAX_COUNT) {
        on = MAX_COUNT; off = 0;                      /* full on  */
    } else {
        on  = (stagger_ && ch < NUM_CHANNELS) ? static_cast<uint16_t>(ch * 256) : 0;
        off = static_cast<uint16_t>((on + duty) & 0x0FFF);
    }
}

esp_err_t Pca9685::writeDuty(uint8_t ch, uint16_t duty)
{
    uint16_t on, off;
    computeOnOff(ch, duty, on, off);
    return writeChannel(ch, on, off);
}

/* ---------- public API ---------- */

esp_err_t Pca9685::setPwmFreq(float freq_hz)
{
    ESP_RETURN_ON_FALSE(dev_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    Guard g(lock_);
    return doSetFreq(freq_hz);
}

esp_err_t Pca9685::getPwmFreq(float &out)
{
    ESP_RETURN_ON_FALSE(dev_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    Guard g(lock_);
    out = freq_hz_;
    return ESP_OK;
}

esp_err_t Pca9685::setPwm(uint8_t ch, uint16_t on, uint16_t off)
{
    ESP_RETURN_ON_FALSE(dev_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(ch <= CHANNEL_ALL && on <= MAX_COUNT && off <= MAX_COUNT,
                        ESP_ERR_INVALID_ARG, TAG, "bad args");
    Guard g(lock_);
    return writeChannel(ch, on, off);
}

esp_err_t Pca9685::setDuty(uint8_t ch, uint16_t duty)
{
    ESP_RETURN_ON_FALSE(dev_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(ch <= CHANNEL_ALL && duty <= MAX_COUNT,
                    ESP_ERR_INVALID_ARG, TAG,
                    "bad args: ch=%u duty=%u (CHANNEL_ALL=%u MAX_COUNT=%u)",
                    ch, duty, (unsigned)CHANNEL_ALL, (unsigned)MAX_COUNT);
    Guard g(lock_);
    return writeDuty(ch, duty);
}

esp_err_t Pca9685::setDutyPercent(uint8_t ch, float percent)
{
    ESP_RETURN_ON_FALSE(percent >= 0.0f && percent <= 100.0f,
                        ESP_ERR_INVALID_ARG, TAG, "percent out of range");
    return setDuty(ch, static_cast<uint16_t>(std::lround(percent * (MAX_COUNT / 100.0f))));
}

esp_err_t Pca9685::setPulseUs(uint8_t ch, float pulse_us)
{
    ESP_RETURN_ON_FALSE(dev_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(ch <= CHANNEL_ALL && pulse_us >= 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "bad args");
    esp_err_t ret;
    {
        Guard g(lock_);
        const long duty = std::lround(pulse_us * freq_hz_ * MAX_COUNT / 1e6f);
        ret = (duty > MAX_COUNT) ? ESP_ERR_INVALID_ARG
                                 : writeDuty(ch, static_cast<uint16_t>(duty));
    }
    if (ret == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "pulse %.1f us longer than the PWM period", pulse_us);
    }
    return ret;
}

esp_err_t Pca9685::sleep()
{
    ESP_RETURN_ON_FALSE(dev_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    Guard g(lock_);
    return doSleep();
}

esp_err_t Pca9685::wake()
{
    ESP_RETURN_ON_FALSE(dev_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    Guard g(lock_);
    return doWake();
}

esp_err_t Pca9685::generalCallReset(i2c_master_bus_handle_t bus)
{
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_INVALID_ARG, TAG, "null bus");

    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address  = 0x00;            /* general call */
    cfg.scl_speed_hz    = 100000;
    i2c_master_dev_handle_t dev;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &dev), TAG, "add general-call dev");

    const uint8_t swrst = 0x06;
    const esp_err_t ret = i2c_master_transmit(dev, &swrst, 1, DEFAULT_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev);
    esp_rom_delay_us(1000);
    return ret;
}

/* ---------- burst write ---------- */
esp_err_t Pca9685::setDuties(uint8_t first_ch, const uint16_t *duty, size_t count)
{
    ESP_RETURN_ON_FALSE(dev_, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(duty && count > 0 && first_ch + count <= NUM_CHANNELS,
                        ESP_ERR_INVALID_ARG, TAG, "bad args");

    uint8_t buf[1 + 4 * NUM_CHANNELS];
    buf[0] = static_cast<uint8_t>(REG_LED0_ON_L + 4 * first_ch);

    for (size_t i = 0; i < count; i++) {
        ESP_RETURN_ON_FALSE(duty[i] <= MAX_COUNT, ESP_ERR_INVALID_ARG, TAG,
                            "ch %u duty out of range", (unsigned)(first_ch + i));
        uint16_t on, off;
        computeOnOff(static_cast<uint8_t>(first_ch + i), duty[i], on, off);
        uint8_t *p = &buf[1 + 4 * i];
        p[0] = on & 0xFF;   p[1] = (on  >> 8) & 0x1F;
        p[2] = off & 0xFF;  p[3] = (off >> 8) & 0x1F;
    }

    Guard g(lock_);
    return i2c_master_transmit(dev_, buf, 1 + 4 * count, timeout_ms_);
}