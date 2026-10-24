#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "base.h"
#include "pid_stabilize.h"
#include "i2c.h"
#include "rtos_utils.h"

static const char *TAG = "main";

static void stabilizer_task(void *arg)
{   
    auto *stab = static_cast<Stabilizer*>(arg);

    constexpr TickType_t period = pdMS_TO_TICKS(5);   // 200 Hz
    int64_t last_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) * 1e-6f;
        last_us = now_us;

        // Guard against a bad dt (first iteration, or a long stall)
        if (dt <= 0.0f || dt > 0.05f) dt = 0.005f;
        
        Feedforward ff;
        if (!ff_box.read(ff, 100000)) ff = Feedforward{}; // stale or missing -> plain PID

        SharedData out{};
        if (stab->update(dt, out, ff) == ESP_OK) {
            out.stamp_us = now_us;          // stamp BEFORE publishing
            rot_box.publish(out);
        } else {
            ESP_LOGW(TAG, "IMU read failed");
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

static void gait_task(void *arg)
{
    auto *base = static_cast<Base*>(arg);

    constexpr TickType_t period = pdMS_TO_TICKS(10);
    int64_t last_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) * 1e-6f;
        last_us = now_us;

        SharedData c{};                                   // identity default
        if (!rot_box.read(c, 30000)) c = SharedData{};    // stale -> no correction

        Feedforward ff{};
        base->update(dt, c, ff);                          // Base fills ff from LegController
        ff.stamp_us = now_us;
        ff_box.publish(ff);

        vTaskDelayUntil(&last_wake, period);
    }
}

extern "C" void app_main()
{
    printf("Initialization Sequence\n");

    static I2CBus i2c_;
    ESP_ERROR_CHECK(i2c_.init());

    static Stabilizer stabilizer_({1.5f, 0.0f, 0.0f});  // TODO - tune
    static Base base_;

    ESP_ERROR_CHECK(base_.init(i2c_.handle()));
    ESP_ERROR_CHECK(stabilizer_.init(i2c_.handle()));
    
    ff_box.init();
    rot_box.init();

    xTaskCreatePinnedToCore(stabilizer_task, "stabilizer", 4096, &stabilizer_, 6, nullptr, 1);
    xTaskCreatePinnedToCore(gait_task,       "gait",       4096, &base_,       5, nullptr, 0);
}