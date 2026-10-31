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
    int64_t stamp_us = 0;          // set by imu_task before publishing
    float   current_speed_mm_s = 0.0f;  // set by Base for the legs (already used)

    Mat3 rot_mat = Mat3::Identity();

    // IMU, written by IMUWrapper
    Vec3 angles_rad { 0.0f, 0.0f, 0.0f};
    Vec3 rpy_rad  {0.0f, 0.0f, 0.0f};  // RADIANS, body orientation in the world
    Vec3 gyro_rad_s {0.0f, 0.0f, 0.0f};  // rad/s, body frame, gyro bias removed
    bool valid = false;            // false in a default SharedData{}: gait_task passes
                                   // that when the mailbox is stale, so Base sees "no IMU"
};

#endif