#ifndef IMU_MAILBOX_h
#define IMU_MAILBOX_h

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "imu_sample.h"

// Single-slot "latest value" mailbox: the IMU task overwrites, the control task peeks.
// Peeking never consumes the sample, so the control loop always sees the newest one.
// If you already have a mailbox type in rtos_utils.h, use that instead and just
// change the publish call in Stabilizer::update().
class ImuMailbox {
    public:
        ImuMailbox() { q_ = xQueueCreate(1, sizeof(ImuSample)); }

        // Producer (IMU task)
        void publish(const ImuSample& s) { xQueueOverwrite(q_, &s); }

        // Consumer (control task). Returns false if nothing has been published yet.
        bool peek(ImuSample& out) const { return xQueuePeek(q_, &out, 0) == pdTRUE; }

    private:
        QueueHandle_t q_ = nullptr;
    };

#endif