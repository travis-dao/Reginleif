#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "pca9685.h"

static const char *TAG = "pca9685";

/* Registers */
#define REG_MODE1          0x00
#define REG_MODE2          0x01
#define REG_LED0_ON_L      0x06
#define REG_ALL_LED_ON_L   0xFA
#define REG_PRESCALE       0xFE

/* MODE1 bits */
#define MODE1_RESTART      0x80
#define MODE1_AI           0x20   /* register auto-increment */
#define MODE1_SLEEP        0x10

/* MODE2 bits */
#define MODE2_INVRT        0x10
#define MODE2_OUTDRV       0x04

#define PRESCALE_MIN       3
#define PRESCALE_MAX       255
#define DEFAULT_FREQ_HZ    200.0f
#define DEFAULT_TIMEOUT_MS 100

struct pca9685_t {
    i2c_master_dev_handle_t dev;
    SemaphoreHandle_t       lock;
    uint32_t                osc_hz;
    float                   freq_hz;
    int                     timeout_ms;
    bool                    stagger;
};

/* ---------- low-level I2C helpers (caller holds the lock) ---------- */

static esp_err_t reg_write(pca9685_handle_t h, uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[5];
    if (len > sizeof(buf) - 1) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(h->dev, buf, len + 1, h->timeout_ms);
}

static esp_err_t reg_write8(pca9685_handle_t h, uint8_t reg, uint8_t val)
{
    return reg_write(h, reg, &val, 1);
}

static esp_err_t reg_read8(pca9685_handle_t h, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(h->dev, &reg, 1, val, 1, h->timeout_ms);
}

static inline void lock(pca9685_handle_t h)   { xSemaphoreTake(h->lock, portMAX_DELAY); }
static inline void unlock(pca9685_handle_t h) { xSemaphoreGive(h->lock); }

/* ---------- power state ---------- */

static esp_err_t do_sleep(pca9685_handle_t h)
{
    uint8_t m1;
    ESP_RETURN_ON_ERROR(reg_read8(h, REG_MODE1, &m1), TAG, "read MODE1");
    if (m1 & MODE1_SLEEP) {
        return ESP_OK;
    }
    /* Never write RESTART=1 while going to sleep, or the restart state is lost */
    m1 = (uint8_t)((m1 & ~MODE1_RESTART) | MODE1_SLEEP);
    return reg_write8(h, REG_MODE1, m1);
}

static esp_err_t do_wake(pca9685_handle_t h)
{
    uint8_t m1;
    ESP_RETURN_ON_ERROR(reg_read8(h, REG_MODE1, &m1), TAG, "read MODE1");

    bool restart = m1 & MODE1_RESTART;
    if (!(m1 & MODE1_SLEEP) && !restart) {
        return ESP_OK;
    }

    m1 &= (uint8_t)~(MODE1_SLEEP | MODE1_RESTART);
    ESP_RETURN_ON_ERROR(reg_write8(h, REG_MODE1, m1), TAG, "clear SLEEP");
    esp_rom_delay_us(500);                 /* oscillator stabilisation (datasheet: 500 us) */
    if (restart) {
        ESP_RETURN_ON_ERROR(reg_write8(h, REG_MODE1, m1 | MODE1_RESTART), TAG, "set RESTART");
    }
    return ESP_OK;
}

/* ---------- frequency ---------- */

static esp_err_t do_set_freq(pca9685_handle_t h, float freq_hz)
{
    if (!(freq_hz >= PCA9685_FREQ_MIN_HZ && freq_hz <= PCA9685_FREQ_MAX_HZ)) {
        return ESP_ERR_INVALID_ARG;
    }
    double p = round((double)h->osc_hz / (4096.0 * (double)freq_hz)) - 1.0;
    if (p < PRESCALE_MIN || p > PRESCALE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t prescale = (uint8_t)p;

    uint8_t m1;
    ESP_RETURN_ON_ERROR(reg_read8(h, REG_MODE1, &m1), TAG, "read MODE1");
    bool was_asleep = m1 & MODE1_SLEEP;

    /* PRE_SCALE is only writable while SLEEP = 1 */
    ESP_RETURN_ON_ERROR(do_sleep(h), TAG, "sleep");
    ESP_RETURN_ON_ERROR(reg_write8(h, REG_PRESCALE, prescale), TAG, "write PRE_SCALE");
    if (!was_asleep) {
        ESP_RETURN_ON_ERROR(do_wake(h), TAG, "wake");
    }

    h->freq_hz = (float)h->osc_hz / (4096.0f * (float)(prescale + 1));
    return ESP_OK;
}

/* ---------- channel write ---------- */

static esp_err_t write_channel(pca9685_handle_t h, uint8_t ch, uint16_t on, uint16_t off)
{
    uint8_t reg = (ch == PCA9685_CHANNEL_ALL) ? REG_ALL_LED_ON_L
                                              : (uint8_t)(REG_LED0_ON_L + 4 * ch);
    /* Bit 12 (0x1000) lands in bit 4 of the *_H register = full-on / full-off flag */
    uint8_t d[4] = {
        (uint8_t)(on & 0xFF),  (uint8_t)((on >> 8) & 0x1F),
        (uint8_t)(off & 0xFF), (uint8_t)((off >> 8) & 0x1F),
    };
    return reg_write(h, reg, d, sizeof(d));
}

static esp_err_t write_duty(pca9685_handle_t h, uint8_t ch, uint16_t duty)
{
    uint16_t on, off;
    if (duty == 0) {
        on = 0;
        off = PCA9685_MAX_COUNT;                 /* full off */
    } else if (duty >= PCA9685_MAX_COUNT) {
        on = PCA9685_MAX_COUNT;                  /* full on  */
        off = 0;
    } else {
        on  = (h->stagger && ch < PCA9685_NUM_CHANNELS) ? (uint16_t)(ch * 256) : 0;
        off = (uint16_t)((on + duty) & 0x0FFF);
    }
    return write_channel(h, ch, on, off);
}

/* ---------- public API ---------- */

esp_err_t pca9685_new(const pca9685_config_t *cfg, pca9685_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->bus && out, ESP_ERR_INVALID_ARG, TAG, "bad args");

    uint8_t  addr    = cfg->addr ? cfg->addr : PCA9685_I2C_ADDR_DEFAULT;
    uint32_t timeout = cfg->timeout_ms ? cfg->timeout_ms : DEFAULT_TIMEOUT_MS;
    float    freq    = cfg->pwm_freq_hz > 0 ? cfg->pwm_freq_hz : DEFAULT_FREQ_HZ;

    ESP_RETURN_ON_ERROR(i2c_master_probe(cfg->bus, addr, timeout), TAG,
                        "no device answering at 0x%02X", addr);

    esp_err_t ret = ESP_OK;
    pca9685_handle_t h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "no mem");

    h->osc_hz     = cfg->osc_clock_hz ? cfg->osc_clock_hz : PCA9685_OSC_HZ_DEFAULT;
    h->timeout_ms = (int)timeout;
    h->stagger    = cfg->stagger_outputs;
    h->lock       = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(h->lock, ESP_ERR_NO_MEM, err, TAG, "no mem for mutex");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = cfg->scl_speed_hz ? cfg->scl_speed_hz : 400000,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(cfg->bus, &dev_cfg, &h->dev),
                      err, TAG, "add device");

    /* Configure while asleep so PRE_SCALE can be written */
    uint8_t mode2 = (cfg->invert_output ? MODE2_INVRT : 0) |
                    (cfg->open_drain    ? 0 : MODE2_OUTDRV);
    ESP_GOTO_ON_ERROR(reg_write8(h, REG_MODE2, mode2), err, TAG, "write MODE2");
    ESP_GOTO_ON_ERROR(reg_write8(h, REG_MODE1, MODE1_SLEEP | MODE1_AI), err, TAG, "write MODE1");

    ESP_GOTO_ON_ERROR(write_channel(h, PCA9685_CHANNEL_ALL, 0, PCA9685_MAX_COUNT),
                      err, TAG, "all off");
    ESP_GOTO_ON_ERROR(do_set_freq(h, freq), err, TAG, "set freq");
    ESP_GOTO_ON_ERROR(do_wake(h), err, TAG, "wake");

    *out = h;
    return ESP_OK;

err:
    pca9685_del(h);
    return ret;
}

esp_err_t pca9685_del(pca9685_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    if (h->dev) {
        i2c_master_bus_rm_device(h->dev);
    }
    if (h->lock) {
        vSemaphoreDelete(h->lock);
    }
    free(h);
    return ESP_OK;
}

esp_err_t pca9685_set_pwm_freq(pca9685_handle_t h, float freq_hz)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    lock(h);
    esp_err_t ret = do_set_freq(h, freq_hz);
    unlock(h);
    return ret;
}

esp_err_t pca9685_get_pwm_freq(pca9685_handle_t h, float *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "bad args");
    lock(h);
    *out = h->freq_hz;
    unlock(h);
    return ESP_OK;
}

esp_err_t pca9685_set_pwm(pca9685_handle_t h, uint8_t ch, uint16_t on, uint16_t off)
{
    ESP_RETURN_ON_FALSE(h && ch <= PCA9685_CHANNEL_ALL &&
                        on <= PCA9685_MAX_COUNT && off <= PCA9685_MAX_COUNT,
                        ESP_ERR_INVALID_ARG, TAG, "bad args");
    lock(h);
    esp_err_t ret = write_channel(h, ch, on, off);
    unlock(h);
    return ret;
}

esp_err_t pca9685_set_duty(pca9685_handle_t h, uint8_t ch, uint16_t duty)
{
    ESP_RETURN_ON_FALSE(h && ch <= PCA9685_CHANNEL_ALL && duty <= PCA9685_MAX_COUNT,
                        ESP_ERR_INVALID_ARG, TAG, "bad args");
    lock(h);
    esp_err_t ret = write_duty(h, ch, duty);
    unlock(h);
    return ret;
}

esp_err_t pca9685_set_duty_percent(pca9685_handle_t h, uint8_t ch, float percent)
{
    ESP_RETURN_ON_FALSE(percent >= 0.0f && percent <= 100.0f,
                        ESP_ERR_INVALID_ARG, TAG, "percent out of range");
    return pca9685_set_duty(h, ch, (uint16_t)lroundf(percent * (PCA9685_MAX_COUNT / 100.0f)));
}

esp_err_t pca9685_set_pulse_us(pca9685_handle_t h, uint8_t ch, float pulse_us)
{
    ESP_RETURN_ON_FALSE(h && ch <= PCA9685_CHANNEL_ALL && pulse_us >= 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "bad args");
    lock(h);
    long duty = lroundf(pulse_us * h->freq_hz * PCA9685_MAX_COUNT / 1e6f);
    esp_err_t ret = (duty > PCA9685_MAX_COUNT) ? ESP_ERR_INVALID_ARG
                                               : write_duty(h, ch, (uint16_t)duty);
    unlock(h);
    if (ret == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "pulse %.1f us longer than the PWM period", pulse_us);
    }
    return ret;
}

esp_err_t pca9685_sleep(pca9685_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    lock(h);
    esp_err_t ret = do_sleep(h);
    unlock(h);
    return ret;
}

esp_err_t pca9685_wake(pca9685_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    lock(h);
    esp_err_t ret = do_wake(h);
    unlock(h);
    return ret;
}

esp_err_t pca9685_general_call_reset(i2c_master_bus_handle_t bus)
{
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_INVALID_ARG, TAG, "null bus");

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = 0x00,           /* general call */
        .scl_speed_hz    = 100000,
    };
    i2c_master_dev_handle_t dev;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &dev), TAG, "add general-call dev");

    const uint8_t swrst = 0x06;
    esp_err_t ret = i2c_master_transmit(dev, &swrst, 1, DEFAULT_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev);
    esp_rom_delay_us(1000);
    return ret;
}