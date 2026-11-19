#pragma once
// Controls a bank of 12 ST3215 servos on one half-duplex bus.

#include <cstdint>
#include "servo.h"

class ServoDriver {
    public:
        static constexpr uint8_t NUM_SERVOS = 12;

        // Servos get IDs firstId .. firstId+11 unless you call setIds().
        explicit ServoDriver(const ST3215Config& cfg = ST3215Config(), uint8_t firstId = 1);

        // Starts the UART, pings every servo, sets position mode, holds each servo at
        // its current position, then enables torque (no jump on power-up).
        // Returns true only if all 12 servos responded. Servos that did not respond
        // are listed in missingMask() (bit i = servo i).
        bool init();

        // angles: 12 values in degrees, one per servo (index 0 -> first ID).
        // Sent as a single sync-write packet, so all servos start moving together.
        bool setAngles(const float angles[NUM_SERVOS]);

        // Reads present angles (degrees, after offset/invert are undone).
        bool readAngles(float angles[NUM_SERVOS]);
        bool readFeedback(uint8_t id, Feedback& fb);

        bool enableTorque(bool on);   // all servos (broadcast)

        // ---- Tuning (optional) ----
        void setSpeed(uint16_t stepsPerSec) { speed_ = stepsPerSec; }   // 0..3400
        void setAcc(uint8_t acc)            { acc_ = acc; }             // 0..254
        void setIds(const uint8_t ids[NUM_SERVOS]);
        void setOffset(uint8_t index, float degrees);    // added after inversion
        void setInverted(uint8_t index, bool inverted);

        uint16_t missingMask() const { return missing; }
        ST3215&  bus() { return servo_; }

    private:
        ST3215   servo_;
        uint8_t  ids_[NUM_SERVOS];
        uint16_t speed_ = 1000;
        uint8_t  acc_   = 50;
        uint16_t missing = 0;

        uint16_t angleToPos(uint8_t i, float deg) const;
        float    posToAngle(uint8_t i, uint16_t pos) const;
};