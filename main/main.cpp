#include <stdint.h>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include <base.h>

Base base;

static void control_task(void *arg)
{
    int64_t last_time_us = esp_timer_get_time();

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        int64_t now_us = esp_timer_get_time();
        int64_t dt_us = now_us - last_time_us;
        last_time_us = now_us;

        double dt_s = dt_us * 1e-6;
        // base.update(dt_s);
        base.drive_servo(dt_s, 3, 0, 180);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main()
{
    base.init();

    // calls base::update() at set interval
    // xTaskCreate(control_task, "control", 4096, nullptr, 5, nullptr);
}
