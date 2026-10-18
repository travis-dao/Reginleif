#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "base.h"
#include "pid_stabilize.h"
#include "i2c.h"
#include "types.h"

static const char *TAG = "main";

static QueueHandle_t correction_q = nullptr;

struct StabilizerCtx {
    Stabilizer *stab;
    QueueHandle_t q;
};

static void stabilizer_task(void *arg)
{
    auto *ctx = static_cast<StabilizerCtx*>(arg);

    constexpr TickType_t period = pdMS_TO_TICKS(5);   // 200 Hz
    int64_t last_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) * 1e-6f;
        last_us = now_us;

        // Guard against a bad dt (first iteration, or a long stall)
        if (dt <= 0.0f || dt > 0.05f) dt = 0.005f;

        // Read IMU, run filter + PID. Adapt to your Stabilizer API.
        SharedData c{};
        if (ctx->stab->update(dt, c) == ESP_OK) {
            xQueueOverwrite(ctx->q, &c);   // never blocks, latest value wins
        } else {
            ESP_LOGW(TAG, "IMU read failed");
            // On repeated failures, consider publishing a zero correction
            // so the robot doesn't keep acting on stale data.
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

static void gait_task(void *arg)
{
    Base *base = static_cast<Base*>(arg);

    constexpr TickType_t period = pdMS_TO_TICKS(10);  // 100 Hz
    int64_t last_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) * 1e-6f;
        last_us = now_us;

        SharedData c{};
        xQueuePeek(correction_q, &c, 0);   // latest value; stays zeroed if none yet

        base->update(dt, c);

        vTaskDelayUntil(&last_wake, period);
    }
}

extern "C" void app_main()
{
    static I2CBus i2c_;
    ESP_ERROR_CHECK(i2c_.init());

    static Stabilizer stabilizer_({1.5f, 0.0f, 0.0f});  // Kp, Ki, Kd
    static Base base_;

    ESP_ERROR_CHECK(base_.init(i2c_.handle()));
    ESP_ERROR_CHECK(stabilizer_.init(i2c_.handle()));

    correction_q = xQueueCreate(1, sizeof(SharedData));
    configASSERT(correction_q);

    static StabilizerCtx stab_ctx{&stabilizer_, correction_q};

    // Stabilizer: higher priority, core 1 (away from WiFi/BT on core 0)
    xTaskCreatePinnedToCore(stabilizer_task, "stabilizer", 4096, &stab_ctx, 6, nullptr, 1);
    // Gait: lower priority
    xTaskCreatePinnedToCore(gait_task, "gait", 4096, &base_, 5, nullptr, 0);
}