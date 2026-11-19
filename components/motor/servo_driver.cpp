#include "servo_driver.h"
#include <cmath>
#include "esp_log.h"

static const char* TAG = "ServoDriver";

ServoDriver::ServoDriver(const ST3215Config& cfg, uint8_t firstId) : servo_(cfg) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) ids_[i] = firstId + i;
}

void ServoDriver::setIds(const uint8_t ids[NUM_SERVOS]) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) ids_[i] = ids[i];
}

uint16_t ServoDriver::angleToPos(uint8_t i, float deg) const {
    deg = fmodf(deg, 360.0f);
    if (deg < 0) deg += 360.0f;
    long pos = lroundf(deg * ST3215::STEPS_PER_REV / 360.0f);
    return (uint16_t)(pos % ST3215::STEPS_PER_REV);
}

float ServoDriver::posToAngle(uint8_t i, uint16_t pos) const {
    float a = pos * 360.0f / ST3215::STEPS_PER_REV;
    a = fmodf(a, 360.0f);
    if (a < 0) a += 360.0f;
    return a;
}

bool ServoDriver::init() {
    esp_err_t err = servo_.begin();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(err));
        return false;
    }

    missing = 0;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        const uint8_t id = ids_[i];

        bool found = false;
        for (int attempt = 0; attempt < 3 && !found; attempt++) found = servo_.ping(id);
        if (!found) {
            ESP_LOGW(TAG, "Servo %u (ID %u) not responding", i, id);
            missing |= (1u << i);
            continue;
        }

        // Mode can only be changed safely with torque off.
        servo_.enableTorque(id, false);
        servo_.setMode(id, ST3215::MODE_POSITION);

        // Hold current position so enabling torque doesn't cause a jump.
        int pos = servo_.readPosition(id);
        if (pos >= 0) servo_.moveTo(id, (uint16_t)pos, speed_, acc_);
        servo_.enableTorque(id, true);
    }

    if (missing) {
        ESP_LOGE(TAG, "%d of %d servos missing (mask 0x%03X)", __builtin_popcount(missing), NUM_SERVOS, missing);
        return false;
    }
    ESP_LOGI(TAG, "All %d servos initialised", NUM_SERVOS);
    return true;
}

bool ServoDriver::setAngles(const float angles[NUM_SERVOS]) {
    uint16_t pos[NUM_SERVOS];
    uint16_t spd[NUM_SERVOS];
    uint8_t  acc[NUM_SERVOS];
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        pos[i] = angleToPos(i, angles[i]);
        spd[i] = speed_;
        acc[i] = acc_;
    }
    return servo_.syncMove(ids_, pos, spd, acc, NUM_SERVOS);
}

bool ServoDriver::readAngles(float angles[NUM_SERVOS]) {
    bool ok = true;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        int p = servo_.readPosition(ids_[i]);
        if (p < 0) { ok = false; angles[i] = NAN; }
        else       angles[i] = posToAngle(i, (uint16_t)p);
    }
    return ok;
}

bool ServoDriver::readFeedback(uint8_t id, Feedback& fb) {
    return servo_.readFeedback(id, fb);
}

bool ServoDriver::enableTorque(bool on) {
    return servo_.writeReg8(ST3215::BROADCAST_ID, ST3215::REG_TORQUE_ENABLE, on ? 1 : 0);
}