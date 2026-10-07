# PCA9685 Driver for ESP-IDF

ESP-IDF driver for the **PCA9685** 16-channel, 12-bit PWM / servo controller (I2C).
Ported from [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos) and written in the style of [esp-idf-lib](https://github.com/UncleRus/esp-idf-lib).

- Author: Ruslan V. Uss
- License: BSD (3-clause)

## Features

- 16 independent PWM channels, 12-bit resolution (0..4096 steps)
- Adjustable PWM frequency (about 24 Hz to 1526 Hz)
- Set one channel, all channels at once, or a block of channels in a single I2C transaction
- Hardware "full on" / "full off" support for exact 0% and 100% duty
- Output inversion and open-drain / totem-pole output mode
- Sleep / wake and restart support
- I2C subaddress support (3 subaddresses)
- Thread-safe: each device descriptor has its own mutex

## Dependencies

- ESP-IDF
- `i2cdev` (from esp-idf-lib)
- `esp_idf_lib_helpers` (from esp-idf-lib)

## Installation

Copy `pca9685.c` and `pca9685.h` into a component directory (for example `components/pca9685/`) and make sure `i2cdev` and `esp_idf_lib_helpers` are also available as components.

Example `CMakeLists.txt` for the component:

```cmake
idf_component_register(
    SRCS pca9685.c
    INCLUDE_DIRS .
    REQUIRES driver esp_idf_lib_helpers i2cdev log
)
```

## Quick start

```c
#include <string.h>
#include <i2cdev.h>
#include <pca9685.h>

#define SDA_GPIO 21
#define SCL_GPIO 22

static i2c_dev_t pca;

void app_main(void)
{
    ESP_ERROR_CHECK(i2cdev_init());

    memset(&pca, 0, sizeof(pca));
    ESP_ERROR_CHECK(pca9685_init_desc(&pca, PCA9685_ADDR_BASE, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(pca9685_init(&pca));

    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&pca, 1000)); // 1 kHz

    // 25% duty on channel 0
    ESP_ERROR_CHECK(pca9685_set_pwm_value(&pca, 0, PCA9685_MAX_PWM_VALUE / 4));

    // ... later
    ESP_ERROR_CHECK(pca9685_free_desc(&pca));
}
```

The descriptor must be zero-initialised (`memset`) before calling `pca9685_init_desc()`.

## Driving servos

Hobby servos expect a 50 Hz signal with a pulse width of roughly 1 ms to 2 ms (many accept 0.5 ms to 2.5 ms). The PCA9685 has 4096 ticks per period, so at 50 Hz (20 ms) one tick is about 4.88 µs:

```
ticks = pulse_us * 4096 * 50 / 1_000_000  ≈  pulse_us * 0.2048
```

| Pulse width | Ticks (approx.) |
|-------------|-----------------|
| 1000 µs     | 205             |
| 1500 µs     | 307             |
| 2000 µs     | 410             |

```c
static uint16_t servo_us_to_ticks(uint16_t us)
{
    return (uint32_t)us * PCA9685_MAX_PWM_VALUE * 50 / 1000000;
}

static esp_err_t servo_write_us(i2c_dev_t *dev, uint8_t channel, uint16_t us)
{
    return pca9685_set_pwm_value(dev, channel, servo_us_to_ticks(us));
}

// Setup
pca9685_set_pwm_frequency(&pca, 50);

// Center servo on channel 3
servo_write_us(&pca, 3, 1500);
```

Calibrate the min/max pulse widths for each servo model to avoid driving against the end stops.

Power note: do not power servos from the ESP32 3.3 V rail. Use a separate 5-6 V supply on the PCA9685 `V+` terminal and share ground with the ESP32.

## API reference

All functions return `esp_err_t` (`ESP_OK` on success, `ESP_ERR_INVALID_ARG` for bad arguments, or an I2C error).

### Constants and types

| Name | Value / description |
|------|---------------------|
| `PCA9685_ADDR_BASE` | `0x40`, base I2C address (address pins add to this) |
| `PCA9685_MAX_PWM_VALUE` | `4096`, one full PWM period |
| `pca9685_channel_t` | Enum `PCA9685_CHANNEL_0` .. `PCA9685_CHANNEL_15`, plus `PCA9685_CHANNEL_ALL` |

### Setup

#### `pca9685_init_desc(dev, addr, port, sda_gpio, scl_gpio)`
Initialise the device descriptor and create its mutex. On ESP32 the I2C clock is set to 1 MHz.

#### `pca9685_free_desc(dev)`
Delete the descriptor's mutex. Call when finished with the device.

#### `pca9685_init(dev)`
Enable register auto-increment (MODE1 `AI` bit). Call once after `pca9685_init_desc()`.

### Power and state

#### `pca9685_restart(dev)`
Restart PWM outputs after sleep, preserving the previous channel states (datasheet 7.3.1.1).

#### `pca9685_is_sleeping(dev, &sleeping)`
Read the sleep state.

#### `pca9685_sleep(dev, sleep)`
`true` puts the device in low-power mode (oscillator off); `false` wakes it and waits 500 µs for the oscillator to stabilise.

### Output configuration

#### `pca9685_is_output_inverted(dev, &inv)` / `pca9685_set_output_inverted(dev, inverted)`
Get or set logical inversion of all outputs (datasheet 7.7).

#### `pca9685_get_output_open_drain(dev, &od)` / `pca9685_set_output_open_drain(dev, od)`
Get or set the output mode. `true` = open-drain, `false` = totem-pole.

### Frequency

#### `pca9685_set_pwm_frequency(dev, freq)` / `pca9685_get_pwm_frequency(dev, &freq)`
Set or read the PWM frequency in Hz. The value is derived from the prescaler:

```
freq = 25 MHz / (4096 * (prescaler + 1))
```

Valid prescaler values are `3..255`, so valid frequencies are about 24 Hz to 1526 Hz. The setter rounds to the nearest prescaler.

#### `pca9685_set_prescaler(dev, prescaler)` / `pca9685_get_prescaler(dev, &prescaler)`
Direct access to the prescaler register. Setting it briefly puts the device to sleep and then wakes it.

### PWM output

#### `pca9685_set_pwm_value(dev, channel, val)`
Set the duty of one channel.

| Parameter | Description |
|-----------|-------------|
| `channel` | `0..15`, or `PCA9685_CHANNEL_ALL` (16) for all channels |
| `val` | `0..4096`. `0` = fully off, `4096` = fully on, in between = number of "on" ticks |

#### `pca9685_set_pwm_values(dev, first_ch, channels, values)`
Set several consecutive channels in one I2C write.

| Parameter | Description |
|-----------|-------------|
| `first_ch` | First channel, `0..15` |
| `channels` | Number of channels to update |
| `values` | Array of duty values, each `0..4096` |

See *Known issues* below before using this function with `first_ch > 0`.

### Subaddresses

#### `pca9685_set_subaddr(dev, num, subaddr, enable)`
Configure one of the three I2C subaddresses (`num` = `0..2`, `subaddr` = 7-bit address) and enable or disable it. See datasheet section 7.3.6.

## Hardware notes

- I2C address is `0x40` plus the A0..A5 address pin bits.
- The `OE` pin is active low. Tie it to ground to enable outputs, or drive it from a GPIO to disable all outputs.
- The internal oscillator runs at 25 MHz (nominal). Actual PWM frequency and servo pulse widths can drift by a few percent between chips, so calibrate if you need accuracy.
- Pull-ups on SDA/SCL are required (most breakout boards include them).

## Known issues

These were found while reviewing the source and are worth fixing before relying on the affected paths:

1. **`pca9685_set_pwm_values()` with `first_ch > 0`.** The internal buffer holds `channels * 4` bytes, but it is indexed with the absolute channel number, which overruns the buffer when `first_ch > 0`. The `values` array is also indexed by absolute channel rather than from 0. Workaround: call it with `first_ch = 0`, or use `pca9685_set_pwm_value()` per channel. The function also does not check `dev` for `NULL`.
2. **`pca9685_set_pwm_frequency()` has no argument validation.** `freq == 0` causes a divide by zero and `dev` is not checked for `NULL`.
3. **Channel documentation mismatch.** The header says "0..15 or >15 for all channels", but the implementation only accepts values up to `PCA9685_CHANNEL_ALL` (16).
4. **`pca9685_set_prescaler()` always wakes the device** afterwards, even if it was asleep before the call.

## License

BSD 3-Clause. Copyright (c) 2016 Ruslan V. Uss. See the license header in the source files for full terms.