/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <i2cManager.h/cpp>

    Manages i2c data transfers so that idle time can be used in our main loop.

*/

#ifndef i2cManager_h
#define i2cManager_h

#include "Arduino.h"
#include <functional>

class CallbackProcessor {
   public:
    virtual void processCallback(uint8_t count, uint8_t *data);
};

struct I2CTransfer {
    uint8_t address;
    uint8_t send_count;
    uint8_t *send_data;
    uint8_t receive_count;
    uint8_t *receive_data;
    CallbackProcessor *cb_object;
    I2CTransfer *next;
};

class I2CManager {
   public:
    I2CManager() {
        waiting_for_data = false;
        transfer = NULL;
    };

    void update();
    void addTransfer(uint8_t address, uint8_t send_count, uint8_t *send_data, uint8_t receive_count, uint8_t *receive_data, CallbackProcessor *cb_object);

    uint8_t readByte(uint8_t address, uint8_t subAddress);
    uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);
    uint8_t writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

   private:
    I2CTransfer *transfer;
    bool waiting_for_data;

};  // class I2CManager

#endif
