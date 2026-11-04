/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "i2cManager.h"
#include <i2c_t3.h>
#include "debug.h"

I2CManager i2c_;

I2CManager& i2c() {
    return i2c_;
}

void I2CManager::begin() {
    // Without a timeout a stuck bus blocks the main loop forever while the motors hold their last PWM value.
    Wire.setDefaultTimeout(TIMEOUT_USEC);
}

bool I2CManager::addTransfer(uint8_t address, uint8_t send_count, uint8_t* send_data, uint8_t receive_count, uint8_t* receive_data, std::function<void(bool)> callback) {
    if (count_ >= QUEUE_CAPACITY) {
        return false;
    }
    queue_[(head_ + count_) % QUEUE_CAPACITY] = I2CTransfer{address, send_count, send_data, receive_count, receive_data, callback};
    ++count_;
    return true;
}

bool I2CManager::perform(const I2CTransfer& transfer) {
    Wire.beginTransmission(transfer.address);
    Wire.write(transfer.send_data, transfer.send_count);
    if (transfer.receive_count == 0) {
        return Wire.endTransmission() == 0;
    }
    if (Wire.endTransmission() != 0) {
        return false;
    }
    if (Wire.requestFrom(transfer.address, transfer.receive_count) != transfer.receive_count) {
        return false;  // timeout or NACK part way through: never wait for bytes that will not arrive
    }
    for (uint8_t i = 0; i < transfer.receive_count; ++i) {
        transfer.receive_data[i] = Wire.read();
    }
    return true;
}

void I2CManager::recordResult(bool ok) {
    if (ok) {
        consecutive_errors_ = 0;
        return;
    }
    ++error_count_;
    if (++consecutive_errors_ >= ERRORS_BEFORE_BUS_RESET) {
        consecutive_errors_ = 0;
        Wire.resetBus();  // clock out a slave that is holding SDA low
        DebugPrintf("I2C bus reset after repeated errors (%lu total)", (unsigned long)error_count_);
    }
}

bool I2CManager::update() {
    if (count_ == 0) {
        return false;
    }
    // Remove the transfer from the queue before running it, so a failure can never block the queue
    // and a callback may safely queue a new transfer.
    I2CTransfer transfer = queue_[head_];
    queue_[head_] = I2CTransfer{};
    head_ = (head_ + 1) % QUEUE_CAPACITY;
    --count_;

    bool ok = perform(transfer);
    recordResult(ok);
    if (transfer.callback) {
        transfer.callback(ok);
    }
    return ok;
}

uint8_t I2CManager::readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data[1];
    if (readBytes(address, subAddress, 1, data))
        return data[0];
    else
        return 0;
}

uint8_t I2CManager::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    if (Wire.endTransmission() != 0) {
        recordResult(false);
        return 0;
    }
    if (Wire.requestFrom(address, count) != count) {
        recordResult(false);
        return 0;
    }
    for (int i = 0; i < count; i++)
        dest[i] = Wire.read();
    recordResult(true);
    return 1;
}

uint8_t I2CManager::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    uint8_t data_write[2];
    data_write[0] = subAddress;
    data_write[1] = data;

    Wire.beginTransmission(address);
    Wire.write(data_write, 2);
    bool ok = Wire.endTransmission() == 0;
    recordResult(ok);
    return ok ? 1 : 0;
}
