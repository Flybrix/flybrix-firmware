/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef i2cManager_h
#define i2cManager_h

#include <array>
#include <functional>
#include "Arduino.h"

struct I2CTransfer {
    uint8_t address;
    uint8_t send_count;
    uint8_t* send_data;
    uint8_t receive_count;
    uint8_t* receive_data;
    // Called exactly once per transfer: true if all bytes were written and read, false on any bus error.
    // Keep captures small (e.g. just `this`) so std::function stores them inline and never allocates.
    std::function<void(bool)> callback;
};

class I2CManager {
   public:
    // Longest a single blocking bus operation may take before it is abandoned.
    // A 14 byte IMU read takes ~450us at 400kHz.
    static constexpr uint32_t TIMEOUT_USEC = 2000;
    // After this many failed transfers in a row the bus is reset (frees a slave holding SDA low).
    static constexpr uint8_t ERRORS_BEFORE_BUS_RESET = 3;
    static constexpr size_t QUEUE_CAPACITY = 8;

    void begin();  // call once after Wire.begin()

    // Performs the oldest queued transfer (blocking, bounded by TIMEOUT_USEC) and runs its callback
    // immediately, so the data is processed as soon as it arrives.
    // Returns true if a transfer completed successfully.
    bool update();

    // Returns false (and does not call the callback) if the queue is full.
    bool addTransfer(uint8_t address, uint8_t send_count, uint8_t* send_data, uint8_t receive_count, uint8_t* receive_data, std::function<void(bool)> callback);

    bool busy() const {
        return count_ > 0;
    }

    uint32_t errorCount() const {
        return error_count_;
    }

    uint8_t readByte(uint8_t address, uint8_t subAddress);
    uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);
    uint8_t writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

   private:
    bool perform(const I2CTransfer& transfer);
    void recordResult(bool ok);

    std::array<I2CTransfer, QUEUE_CAPACITY> queue_{};
    size_t head_{0};
    size_t count_{0};
    uint32_t error_count_{0};
    uint8_t consecutive_errors_{0};
};  // class I2CManager

I2CManager& i2c();

#endif
