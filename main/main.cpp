#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "i2c.h"
#include "base.h"
#include "imu_subsystem.h"

// static const char *TAG = "main";

ImuMailbox imu_mailbox;

static void gait_task(void *arg) {
    auto *base = static_cast<Base*>(arg);

    constexpr TickType_t period = pdMS_TO_TICKS(10);
    int64_t last_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) * 1e-6f;
        last_us = now_us;
        
        ImuSample imu;
        if (!imu_mailbox.peek(imu) || (uint32_t)esp_timer_get_time() - imu.stamp_us > 50000) {
            imu.valid = false;   // nothing published yet, or older than 50 ms
        }

        base->update(dt, imu);


        vTaskDelayUntil(&last_wake, period);
    }
}

extern "C" void app_main() {
    printf("Initialization Sequence\n");

    static I2CBus i2c;
    ESP_ERROR_CHECK(i2c.init());

    static Stabilizer stabilizer(imu_mailbox, Stabilizer::Config{});
    static Base base;

    base.init();
    stabilizer.init(i2c.handle());

    xTaskCreatePinnedToCore(
        Stabilizer::taskEntry, 
        "imu", 
        4096, 
        &stabilizer, 
        6, 
        nullptr, 
        1
    );
    xTaskCreatePinnedToCore(
        gait_task, 
        "motion", 
        4096, 
        &base, 
        5, 
        nullptr, 
        0
    );
}