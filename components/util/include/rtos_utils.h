#ifndef RTOS_UTILS_h
#define RTOS_UTILS_h

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_timer.h"

#include "types.h"

template <typename T>
class Mailbox {
    public:
        void init() { 
            q_ = xQueueCreate(1, sizeof(T));
            configASSERT(q_);
        }
        void publish(const T& v) { 
            xQueueOverwrite(q_, &v); // never blocks
        }
        // Returns false if empty or older than max_age_us
        bool read(T& out, int64_t max_age_us) const {
            if (xQueuePeek(q_, &out, 0) != pdTRUE) {
                return false;
            }

            return (esp_timer_get_time() - out.stamp_us) <= max_age_us;
        }
    private:
        QueueHandle_t q_ = nullptr;
};

struct SharedData {
	Mat3 rot_mat;
	float current_speed;
	int64_t stamp_us = 0;
};

struct Feedforward {
    Vec3     attitude_deg    {0, 0, 0};
    Vec3     rate_dps        {0, 0, 0}; // TODO - tune
    bool     freeze_integral = false;
    int64_t stamp_us        = 0;
};

extern Mailbox<Feedforward> ff_box;
extern Mailbox<SharedData> rot_box;

#endif