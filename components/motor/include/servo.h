#pragma once
// ST3215 (Feetech STS3215) serial bus servo driver for ESP-IDF (C++).
// Half-duplex TTL UART, default 1,000,000 baud 8N1, little-endian registers.

#include <cstdint>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

struct ST3215Config {
    uart_port_t uart     = UART_NUM_1;
    int         tx_pin   = 17;
    int         rx_pin   = 16;
    int         tx_en_pin = -1;   // GPIO that is HIGH while transmitting
                                  // (74HC125 enable / DE pin). -1 = not used
    bool        echo     = false; // true if TX and RX are wired together
    int         baud     = 1000000;
    int         rx_buf   = 512;
    uint16_t    timeout_ms = 20;
};

struct Feedback {
    uint16_t position;     // 0..4095 (4096 = 360 deg)
    int16_t  speed;        // steps/s
    int16_t  load;         // -1000..1000
    uint8_t  voltage;      // 0.1 V units
    uint8_t  temperature;  // deg C
    bool     moving;
    int16_t  current;      // mA (approx)
};

class ST3215 {
    public:
        using Config = ST3215Config;

        enum Reg : uint8_t {
            REG_ID            = 5,
            REG_BAUD          = 6,
            REG_MIN_ANGLE     = 9,    // 2 bytes
            REG_MAX_ANGLE     = 11,   // 2 bytes
            REG_MAX_TEMP      = 13,
            REG_MAX_VOLTAGE   = 14,
            REG_MIN_VOLTAGE   = 15,
            REG_MAX_TORQUE    = 16,
            REG_P_COEF        = 21,
            REG_D_COEF        = 22,
            REG_I_COEF        = 23,
            REG_OFFSET        = 31,
            REG_MODE          = 33,
            REG_TORQUE_ENABLE = 40,   // 0 off, 1 on, 128 = set current pos to 2048
            REG_ACC           = 41,   // block: acc, pos(2), time(2), speed(2)
            REG_GOAL_POS      = 42,
            REG_GOAL_TIME     = 44,
            REG_GOAL_SPEED    = 46,
            REG_TORQUE_LIMIT  = 48,
            REG_LOCK          = 55,   // 0 = EEPROM writable, 1 = locked
            REG_PRESENT_POS   = 56
        };

        enum Mode : uint8_t { MODE_POSITION = 0, MODE_VELOCITY = 1, MODE_PWM = 2, MODE_STEP = 3 };

        enum Baud : uint8_t {
            BAUD_1M = 0, BAUD_500K = 1, BAUD_250K = 2, BAUD_128K = 3,
            BAUD_115200 = 4, BAUD_76800 = 5, BAUD_57600 = 6, BAUD_38400 = 7
        };

        static constexpr uint8_t  BROADCAST_ID  = 0xFE;
        static constexpr uint16_t STEPS_PER_REV = 4096;

        explicit ST3215(const Config& cfg = Config());

        esp_err_t begin();            // installs and configures the UART driver
        void      end();
        uint8_t   lastError() const { return last_error_; }

        // ---- Basic ----
        bool ping(uint8_t id);

        // ---- Motion ----
        bool moveTo(uint8_t id, uint16_t position, uint16_t speed = 1000, uint8_t acc = 50);
        bool moveToDeg(uint8_t id, float degrees, uint16_t speed = 1000, uint8_t acc = 50);
        bool moveToAsync(uint8_t id, uint16_t position, uint16_t speed = 1000, uint8_t acc = 50);
        bool action();
        bool syncMove(const uint8_t* ids, const uint16_t* positions, const uint16_t* speeds,
                    const uint8_t* accs, uint8_t count);
        bool setWheelSpeed(uint8_t id, int16_t speed, uint8_t acc = 50);
        bool stop(uint8_t id);

        // ---- Configuration ----
        bool setMode(uint8_t id, Mode mode);
        bool enableTorque(uint8_t id, bool on);
        bool setMiddlePosition(uint8_t id);
        bool setAngleLimits(uint8_t id, uint16_t minPos, uint16_t maxPos);  // 0,0 = unlimited
        bool setTorqueLimit(uint8_t id, uint16_t limit);
        bool setId(uint8_t id, uint8_t newId);
        bool setBaud(uint8_t id, Baud baud);
        bool unlockEeprom(uint8_t id);
        bool lockEeprom(uint8_t id);

        // ---- Feedback ----
        bool readFeedback(uint8_t id, Feedback& fb);
        int  readPosition(uint8_t id);              // -1 on failure
        int  readSpeed(uint8_t id, bool* ok = nullptr);
        int  readLoad(uint8_t id, bool* ok = nullptr);
        int  readVoltage(uint8_t id);               // 0.1 V, -1 on failure
        int  readTemperature(uint8_t id);           // deg C, -1 on failure
        int  readMoving(uint8_t id);                // 1/0, -1 on failure

        // ---- Raw ----
        bool readReg(uint8_t id, uint8_t addr, uint8_t* data, uint8_t len);
        bool writeReg(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len);
        bool writeReg8(uint8_t id, uint8_t addr, uint8_t v);
        bool writeReg16(uint8_t id, uint8_t addr, uint16_t v);

    private:
        Config  cfg_;
        uint8_t last_error_ = 0;
        bool    installed_ = false;

        void sendPacket(uint8_t id, uint8_t inst, const uint8_t* params, uint8_t n);
        bool receive(uint8_t id, uint8_t* data, uint8_t n);
        bool readExact(uint8_t* buf, uint8_t n, int64_t deadlineUs);
        static int16_t  decodeSigned(uint16_t raw, uint8_t signBit);
        static uint16_t encodeSigned(int16_t v, uint8_t signBit);
};