/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
    *
    *
*/

#include "i2cManager.h"
#include <i2c_t3.h>

I2CTransfer& I2CManager::TransferQueue::front() const {
    return firstItem->item;
}

void I2CManager::TransferQueue::push(I2CTransfer&& newItem) {
    std::unique_ptr<I2CManager::QueueItem>* lastItem = &firstItem;
    while (*lastItem) {
        lastItem = &((*lastItem)->nextItem);
    }
    lastItem->reset(new I2CManager::QueueItem{newItem, nullptr});
}

void I2CManager::TransferQueue::pop() {
    if (empty())
        return;
    firstItem.reset(firstItem->nextItem.release());
}

bool I2CManager::TransferQueue::empty() const {
    return !firstItem;
}

void I2CManager::addTransfer(uint8_t address, uint8_t send_count, uint8_t* send_data, uint8_t receive_count, uint8_t* receive_data, CallbackProcessor* cb_object) {
    transfers.push(I2CTransfer{address, send_count, send_data, receive_count, receive_data, cb_object});
}

void I2CManager::update() {
    if (waiting_for_data) {
        if (Wire.available() == transfers.front().receive_count) {
            for (uint8_t i = 0; i < transfers.front().receive_count; i++)
                transfers.front().receive_data[i] = Wire.read();
            transfers.front().cb_object->triggerCallback();
            transfers.pop();
            waiting_for_data = false;
        }
    } else if (!transfers.empty() && !waiting_for_data) {
        // begin new transfer
        Wire.beginTransmission(transfers.front().address);
        Wire.write(transfers.front().send_data, transfers.front().send_count);
        uint8_t error = Wire.endTransmission();
        if (error == 0 && transfers.front().receive_count > 0) {
            waiting_for_data = true;
            Wire.requestFrom(transfers.front().address, transfers.front().receive_count);
        } else {
            // how do we want to handle errors? ignore for now
        }
    }
}

uint8_t I2CManager::readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data[1];
    if (readBytes(address, subAddress, 1, data))
        return data[0];
    else
        return 0;
}

uint8_t I2CManager::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
    int error;

    Wire.beginTransmission(address);
    Wire.write(subAddress);
    error = Wire.endTransmission();
    if (error == 0) {
        Wire.requestFrom(address, count);
        while (Wire.available() != count)
            ;  // wait until bytes are ready

        for (int i = 0; i < count; i++)
            dest[i] = Wire.read();

        return 1;
    }
    return 0;
}

uint8_t I2CManager::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    int error;

    uint8_t data_write[2];
    data_write[0] = subAddress;
    data_write[1] = data;

    Wire.beginTransmission(address);
    Wire.write(data_write, 2);
    error = Wire.endTransmission();

    if (error == 0)
        return (1);
    else
        return (0);
}
