# MPU6050 Driver for ESP-IDF

A lightweight I2C driver for the InvenSense **MPU6050** 6-axis IMU (3-axis accelerometer + 3-axis gyroscope + temperature sensor), built on the ESP-IDF legacy I2C master driver (`driver/i2c.h`).

## Features

- Create/delete sensor handles on any I2C port
- Wake / sleep control
- Configurable accelerometer (±2/4/8/16 g) and gyroscope (±250/500/1000/2000 °/s) full-scale ranges
- Raw and scaled accelerometer and gyroscope readings
- On-die temperature reading
- Complementary filter producing **roll** and **pitch** angles
- Interrupt support: pin configuration, enabling/disabling sources, ISR registration, status decoding

## Requirements

- ESP-IDF (uses the legacy `driver/i2c.h` API)
- An MPU6050 breakout board (e.g. GY-521)
- `mpu6050.h` — public header declaring the handle, enums, and structs used here

## Wiring

| MPU6050 | ESP32 | Notes |
|---------|-------|-------|
| VCC     | 3V3   | Most breakouts also accept 5 V via an onboard regulator |
| GND     | GND   | |
| SCL     | any GPIO (e.g. 22) | Pull-ups required (usually on the breakout) |
| SDA     | any GPIO (e.g. 21) | Pull-ups required |
| INT     | any GPIO (optional) | Only needed for interrupts |
| AD0     | GND or 3V3 | GND → address `0x68`, 3V3 → address `0x69` |

## Quick Start

```c
#include "driver/i2c.h"
#include "mpu6050.h"

#define I2C_PORT   I2C_NUM_0
#define MPU_ADDR   0x68          // 7-bit address (do NOT pre-shift)

static void i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

void app_main(void)
{
    i2c_bus_init();

    mpu6050_handle_t mpu = mpu6050_create(I2C_PORT, MPU_ADDR);

    uint8_t id = 0;
    ESP_ERROR_CHECK(mpu6050_get_deviceid(mpu, &id));   // expect 0x68

    ESP_ERROR_CHECK(mpu6050_config(mpu, ACCE_FS_4G, GYRO_FS_500DPS));
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu));

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    complimentary_angle_t angle;

    while (1) {
        mpu6050_get_acce(mpu, &acce);
        mpu6050_get_gyro(mpu, &gyro);
        mpu6050_complimentory_filter(mpu, &acce, &gyro, &angle);

        printf("roll: %.2f  pitch: %.2f\n", angle.roll, angle.pitch);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    mpu6050_delete(mpu);
}
```

## API Reference

All functions return `esp_err_t` unless noted.

### Lifecycle

| Function | Description |
|----------|-------------|
| `mpu6050_create(port, dev_addr)` | Allocates a sensor handle. Returns `mpu6050_handle_t`. `dev_addr` is the **7-bit** address; the driver shifts it internally. |
| `mpu6050_delete(sensor)` | Frees the handle (`void`). |
| `mpu6050_get_deviceid(sensor, &id)` | Reads `WHO_AM_I` (0x75). |
| `mpu6050_wake_up(sensor)` | Clears the SLEEP bit in `PWR_MGMT_1`. The device powers up asleep, so call this first. |
| `mpu6050_sleep(sensor)` | Sets the SLEEP bit. |

### Configuration

| Function | Description |
|----------|-------------|
| `mpu6050_config(sensor, acce_fs, gyro_fs)` | Sets accelerometer and gyroscope full-scale ranges. |
| `mpu6050_get_acce_sensitivity(sensor, &s)` | Returns LSB/g for the current range (16384 / 8192 / 4096 / 2048). |
| `mpu6050_get_gyro_sensitivity(sensor, &s)` | Returns LSB/(°/s) for the current range (131 / 65.5 / 32.8 / 16.4). |

### Measurements

| Function | Output |
|----------|--------|
| `mpu6050_get_raw_acce` | Raw signed 16-bit counts (x, y, z) |
| `mpu6050_get_raw_gyro` | Raw signed 16-bit counts (x, y, z) |
| `mpu6050_get_acce` | Acceleration in **g** |
| `mpu6050_get_gyro` | Angular rate in **°/s** |
| `mpu6050_get_temp` | Temperature in **°C** (`raw / 340 + 36.53`) |

The scaled getters read the current range from the device on every call, so each costs an extra I2C transaction.

### Complementary Filter

```c
esp_err_t mpu6050_complimentory_filter(sensor, &acce, &gyro, &angle);
```

Fuses gyroscope integration with accelerometer tilt to produce `angle.roll` and `angle.pitch` (degrees):

```
angle = ALPHA * (angle + gyro_rate * dt) + (1 - ALPHA) * accel_angle
```

- `ALPHA = 0.99` — 99% gyro, 1% accelerometer
- `dt` is measured internally with `gettimeofday()` between calls
- The first call only initializes the angles from the accelerometer and returns
- Call it at a steady, fast rate (a few ms to tens of ms) for good results
- Yaw is **not** estimated (it would drift without a magnetometer)

### Interrupts

| Function | Description |
|----------|-------------|
| `mpu6050_config_interrupts(sensor, &cfg)` | Configures the INT pin polarity, drive mode, latch, and clear behavior on the MPU6050, and sets up the ESP32 GPIO edge type. |
| `mpu6050_register_isr(sensor, isr)` | Attaches an ISR to the INT GPIO and enables the interrupt. Requires `gpio_install_isr_service()` to have been called. |
| `mpu6050_enable_interrupts(sensor, sources)` | Enables one or more sources in `INT_ENABLE`. |
| `mpu6050_disable_interrupts(sensor, sources)` | Disables one or more sources. |
| `mpu6050_get_interrupt_status(sensor, &status)` | Reads `INT_STATUS`. |
| `mpu6050_is_data_ready_interrupt(status)` | Tests the data-ready bit. |
| `mpu6050_is_i2c_master_interrupt(status)` | Tests the I2C-master bit. |
| `mpu6050_is_fifo_overflow_interrupt(status)` | Tests the FIFO-overflow bit. |

Interrupt source masks: `MPU6050_DATA_RDY_INT_BIT`, `MPU6050_I2C_MASTER_INT_BIT`, `MPU6050_FIFO_OVERFLOW_INT_BIT`, `MPU6050_MOT_DETECT_INT_BIT`, or all at once with `MPU6050_ALL_INTERRUPTS`.

Typical flow:

```c
gpio_install_isr_service(0);

mpu6050_int_config_t cfg = {
    .interrupt_pin = GPIO_NUM_5,
    .active_level = INTERRUPT_PIN_ACTIVE_LOW,
    .pin_mode = INTERRUPT_PIN_OPEN_DRAIN,
    .interrupt_latch = INTERRUPT_LATCH_UNTIL_CLEARED,
    .interrupt_clear_behavior = INTERRUPT_CLEAR_ON_ANY_READ,
};
mpu6050_config_interrupts(mpu, &cfg);
mpu6050_register_isr(mpu, my_isr);
mpu6050_enable_interrupts(mpu, MPU6050_DATA_RDY_INT_BIT);
```

In the ISR, keep work minimal (e.g. notify a task); read the sensor from task context, not from the ISR.

## Register Map Used

| Register | Address |
|----------|---------|
| `GYRO_CONFIG` | 0x1B |
| `ACCEL_CONFIG` | 0x1C |
| `INT_PIN_CFG` | 0x37 |
| `INT_ENABLE` | 0x38 |
| `INT_STATUS` | 0x3A |
| `ACCEL_XOUT_H` | 0x3B |
| `TEMP_OUT_H` | 0x41 |
| `GYRO_XOUT_H` | 0x43 |
| `PWR_MGMT_1` | 0x6B |
| `WHO_AM_I` | 0x75 |

## Known Issues / Notes

These are things worth knowing (or fixing) when using this code:

- **`RAD_TO_DEG` is slightly off.** It is defined as `57.27272727f`; the correct value is `57.29577951f` (≈ 0.04% error in angles).
- **Interrupt pin mask overflow.** `BIT0 << interrupt_pin` is a 32-bit shift, so GPIOs ≥ 32 would break. Use `1ULL << pin`.
- **`mpu6050_delete()` leaks the timer.** It frees the handle but not the `timeval` allocated in `mpu6050_create()`; add `free(sens->timer)` before `free(sens)`.
- **`mpu6050_create()` does not check `calloc()` results.**
- **Read errors in raw getters are not guarded.** `mpu6050_get_raw_acce/gyro/temp` convert the buffer even if the I2C read failed; check the return value before trusting the data.
- **Pitch convention.** Pitch is computed as `atan2(x, z)` rather than the more common `atan2(-x, sqrt(y² + z²)`, so it becomes unstable near ±90°.
- **`inline` without `static`** on the interrupt-status helpers can cause linker errors in some build configurations; declare them `static inline` or move them to the header.
- **Mixed-case spelling:** the filter function and its angle struct are spelled `complimentory` / `complimentary` (sic) in the API.
- **Legacy I2C driver.** Newer ESP-IDF versions deprecate `driver/i2c.h` in favor of `driver/i2c_master.h`.

## License

Apache-2.0 — Copyright 2015–2021 Espressif Systems (Shanghai) CO LTD.