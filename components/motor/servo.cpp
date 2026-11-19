#include "servo.h"
#include <cstring>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr uint8_t INST_PING       = 0x01;
constexpr uint8_t INST_READ       = 0x02;
constexpr uint8_t INST_WRITE      = 0x03;
constexpr uint8_t INST_REG_WRITE  = 0x04;
constexpr uint8_t INST_ACTION     = 0x05;
constexpr uint8_t INST_SYNC_WRITE = 0x83;
constexpr uint8_t MAX_SYNC        = 16;
}

ST3215::ST3215(const Config& cfg) : cfg_(cfg) {}

esp_err_t ST3215::begin() {
    if (installed_) return ESP_OK;

    uart_config_t uc = {};
    uc.baud_rate  = cfg_.baud;
    uc.data_bits  = UART_DATA_8_BITS;
    uc.parity     = UART_PARITY_DISABLE;
    uc.stop_bits  = UART_STOP_BITS_1;
    uc.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uc.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(cfg_.uart, cfg_.rx_buf, 0, 0, nullptr, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(cfg_.uart, &uc);
    if (err != ESP_OK) return err;
    err = uart_set_pin(cfg_.uart, cfg_.tx_pin, cfg_.rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    if (cfg_.tx_en_pin >= 0) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << cfg_.tx_en_pin;
        io.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io);
        gpio_set_level((gpio_num_t)cfg_.tx_en_pin, 0);
    }
    installed_ = true;
    return ESP_OK;
}

void ST3215::end() {
    if (installed_) { uart_driver_delete(cfg_.uart); installed_ = false; }
}

// ---------------------------------------------------------------- low level
// Packet: FF FF ID LEN INST [PARAMS...] CHK  (LEN = params + 2)
// CHK = ~(ID + LEN + INST + sum(PARAMS))
void ST3215::sendPacket(uint8_t id, uint8_t inst, const uint8_t* params, uint8_t n) {
    uint8_t buf[6 + 2 + MAX_SYNC * 8];
    if (n > sizeof(buf) - 6) return;

    const uint8_t len = n + 2;
    buf[0] = 0xFF; buf[1] = 0xFF;
    buf[2] = id;   buf[3] = len; buf[4] = inst;
    uint8_t sum = id + len + inst;
    for (uint8_t i = 0; i < n; i++) { buf[5 + i] = params[i]; sum += params[i]; }
    buf[5 + n] = ~sum;
    const uint8_t total = 6 + n;

    uart_flush_input(cfg_.uart);                       // drop stale bytes

    if (cfg_.tx_en_pin >= 0) gpio_set_level((gpio_num_t)cfg_.tx_en_pin, 1);
    uart_write_bytes(cfg_.uart, buf, total);
    uart_wait_tx_done(cfg_.uart, pdMS_TO_TICKS(100));  // last bit is out
    if (cfg_.tx_en_pin >= 0) gpio_set_level((gpio_num_t)cfg_.tx_en_pin, 0);

    if (cfg_.echo) {                                   // discard our own echo
        uint8_t dummy[sizeof(buf)];
        uart_read_bytes(cfg_.uart, dummy, total, pdMS_TO_TICKS(cfg_.timeout_ms));
    }
}

bool ST3215::readExact(uint8_t* buf, uint8_t n, int64_t deadlineUs) {
    uint8_t got = 0;
    while (got < n) {
        int64_t remain = deadlineUs - esp_timer_get_time();
        if (remain <= 0) return false;
        TickType_t ticks = pdMS_TO_TICKS((remain + 999) / 1000);
        if (ticks == 0) ticks = 1;
        int r = uart_read_bytes(cfg_.uart, buf + got, n - got, ticks);
        if (r > 0) got += r;
    }
    return true;
}

// Reply: FF FF ID LEN ERR [DATA...] CHK
bool ST3215::receive(uint8_t id, uint8_t* data, uint8_t n) {
    const int64_t deadline = esp_timer_get_time() + (int64_t)cfg_.timeout_ms * 1000;

    uint8_t hdr = 0, b;
    while (hdr < 2) {
        if (!readExact(&b, 1, deadline)) return false;
        hdr = (b == 0xFF) ? hdr + 1 : 0;
    }

    uint8_t h[3];                                      // ID, LEN, ERR
    if (!readExact(h, 3, deadline)) return false;
    if (h[0] != id || h[1] != n + 2) return false;
    last_error_ = h[2];

    uint8_t sum = h[0] + h[1] + h[2];
    if (n && !readExact(data, n, deadline)) return false;
    for (uint8_t i = 0; i < n; i++) sum += data[i];

    uint8_t chk;
    if (!readExact(&chk, 1, deadline)) return false;
    return chk == (uint8_t)~sum;
}

int16_t ST3215::decodeSigned(uint16_t raw, uint8_t signBit) {
    int16_t mag = raw & ~(1u << signBit);
    return (raw & (1u << signBit)) ? -mag : mag;
}

uint16_t ST3215::encodeSigned(int16_t v, uint8_t signBit) {
    return v < 0 ? ((uint16_t)(-v) | (1u << signBit)) : (uint16_t)v;
}

// ------------------------------------------------------------- raw register
bool ST3215::ping(uint8_t id) {
    sendPacket(id, INST_PING, nullptr, 0);
    return receive(id, nullptr, 0);
}

bool ST3215::readReg(uint8_t id, uint8_t addr, uint8_t* data, uint8_t len) {
    uint8_t p[2] = { addr, len };
    sendPacket(id, INST_READ, p, 2);
    return receive(id, data, len);
}

bool ST3215::writeReg(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len) {
    uint8_t p[1 + 16];
    if (len > 16) return false;
    p[0] = addr;
    memcpy(p + 1, data, len);
    sendPacket(id, INST_WRITE, p, len + 1);
    if (id == BROADCAST_ID) return true;               // broadcast: no reply
    return receive(id, nullptr, 0);
}

bool ST3215::writeReg8(uint8_t id, uint8_t addr, uint8_t v) {
    return writeReg(id, addr, &v, 1);
}

bool ST3215::writeReg16(uint8_t id, uint8_t addr, uint16_t v) {
    uint8_t d[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };  // little-endian
    return writeReg(id, addr, d, 2);
}

// ------------------------------------------------------------------ motion
// Block at 41: ACC(1) POS(2) TIME(2) SPEED(2)
static void packMove(uint8_t* d, uint16_t pos, uint16_t speed, uint8_t acc) {
    if (pos > 4095) pos = 4095;
    if (speed > 3400) speed = 3400;
    if (acc > 254) acc = 254;
    d[0] = acc;
    d[1] = pos & 0xFF;   d[2] = pos >> 8;
    d[3] = 0;            d[4] = 0;                     // time = 0 -> use speed
    d[5] = speed & 0xFF; d[6] = speed >> 8;
}

bool ST3215::moveTo(uint8_t id, uint16_t position, uint16_t speed, uint8_t acc) {
    uint8_t d[7];
    packMove(d, position, speed, acc);
    return writeReg(id, REG_ACC, d, 7);
}

bool ST3215::moveToDeg(uint8_t id, float degrees, uint16_t speed, uint8_t acc) {
    while (degrees < 0) degrees += 360.0f;
    while (degrees >= 360.0f) degrees -= 360.0f;
    return moveTo(id, (uint16_t)(degrees * STEPS_PER_REV / 360.0f), speed, acc);
}

bool ST3215::moveToAsync(uint8_t id, uint16_t position, uint16_t speed, uint8_t acc) {
    uint8_t p[8];
    p[0] = REG_ACC;
    packMove(p + 1, position, speed, acc);
    sendPacket(id, INST_REG_WRITE, p, 8);
    return receive(id, nullptr, 0);
}

bool ST3215::action() {
    sendPacket(BROADCAST_ID, INST_ACTION, nullptr, 0);
    return true;
}

bool ST3215::syncMove(const uint8_t* ids, const uint16_t* positions, const uint16_t* speeds,
                      const uint8_t* accs, uint8_t count) {
    if (count == 0 || count > MAX_SYNC) return false;
    uint8_t p[2 + MAX_SYNC * 8];
    p[0] = REG_ACC;
    p[1] = 7;                                          // bytes per servo
    uint8_t n = 2;
    for (uint8_t i = 0; i < count; i++) {
        p[n++] = ids[i];
        packMove(p + n, positions[i], speeds[i], accs[i]);
        n += 7;
    }
    sendPacket(BROADCAST_ID, INST_SYNC_WRITE, p, n);
    return true;
}

bool ST3215::setWheelSpeed(uint8_t id, int16_t speed, uint8_t acc) {
    if (speed > 3400) speed = 3400;
    if (speed < -3400) speed = -3400;
    if (!writeReg8(id, REG_ACC, acc)) return false;
    return writeReg16(id, REG_GOAL_SPEED, encodeSigned(speed, 15));
}

bool ST3215::stop(uint8_t id) { return setWheelSpeed(id, 0, 0); }

// ----------------------------------------------------------- configuration
bool ST3215::setMode(uint8_t id, Mode mode) { return writeReg8(id, REG_MODE, mode); }

bool ST3215::enableTorque(uint8_t id, bool on) {
    return writeReg8(id, REG_TORQUE_ENABLE, on ? 1 : 0);
}

bool ST3215::setMiddlePosition(uint8_t id) { return writeReg8(id, REG_TORQUE_ENABLE, 128); }

bool ST3215::setAngleLimits(uint8_t id, uint16_t minPos, uint16_t maxPos) {
    if (!unlockEeprom(id)) return false;
    bool ok = writeReg16(id, REG_MIN_ANGLE, minPos) && writeReg16(id, REG_MAX_ANGLE, maxPos);
    return lockEeprom(id) && ok;
}

bool ST3215::setTorqueLimit(uint8_t id, uint16_t limit) {
    if (limit > 1000) limit = 1000;
    return writeReg16(id, REG_TORQUE_LIMIT, limit);
}

bool ST3215::unlockEeprom(uint8_t id) { return writeReg8(id, REG_LOCK, 0); }
bool ST3215::lockEeprom(uint8_t id)   { return writeReg8(id, REG_LOCK, 1); }

bool ST3215::setId(uint8_t id, uint8_t newId) {
    if (newId >= BROADCAST_ID) return false;
    if (!unlockEeprom(id)) return false;
    if (!writeReg8(id, REG_ID, newId)) return false;
    return lockEeprom(newId);                          // servo now answers to newId
}

bool ST3215::setBaud(uint8_t id, Baud baud) {
    if (!unlockEeprom(id)) return false;
    bool ok = writeReg8(id, REG_BAUD, baud);
    return lockEeprom(id) && ok;   // lock may not ack if the baud already changed
}

// ---------------------------------------------------------------- feedback
bool ST3215::readFeedback(uint8_t id, Feedback& fb) {
    uint8_t d[15];                                     // regs 56..70
    if (!readReg(id, REG_PRESENT_POS, d, 15)) return false;
    fb.position    = d[0] | (d[1] << 8);
    fb.speed       = decodeSigned(d[2] | (d[3] << 8), 15);
    fb.load        = decodeSigned(d[4] | (d[5] << 8), 10);
    fb.voltage     = d[6];
    fb.temperature = d[7];
    fb.moving      = d[10] != 0;
    fb.current     = (int16_t)((d[13] | (d[14] << 8)) * 6.5f);
    return true;
}

int ST3215::readPosition(uint8_t id) {
    uint8_t d[2];
    return readReg(id, REG_PRESENT_POS, d, 2) ? (d[0] | (d[1] << 8)) : -1;
}

int ST3215::readSpeed(uint8_t id, bool* ok) {
    uint8_t d[2];
    bool r = readReg(id, 58, d, 2);
    if (ok) *ok = r;
    return r ? decodeSigned(d[0] | (d[1] << 8), 15) : 0;
}

int ST3215::readLoad(uint8_t id, bool* ok) {
    uint8_t d[2];
    bool r = readReg(id, 60, d, 2);
    if (ok) *ok = r;
    return r ? decodeSigned(d[0] | (d[1] << 8), 10) : 0;
}

int ST3215::readVoltage(uint8_t id) {
    uint8_t v;
    return readReg(id, 62, &v, 1) ? v : -1;
}

int ST3215::readTemperature(uint8_t id) {
    uint8_t v;
    return readReg(id, 63, &v, 1) ? v : -1;
}

int ST3215::readMoving(uint8_t id) {
    uint8_t v;
    return readReg(id, 66, &v, 1) ? (v ? 1 : 0) : -1;
}