/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef i2cManager_h
#define i2cManager_h

#include <functional>
#include <memory>
#include "Arduino.h"

struct I2CTransfer {
    uint8_t address;
    uint8_t send_count;
    uint8_t* send_data;
    uint8_t receive_count;
    uint8_t* receive_data;
    std::function<void()> callback;
};

class I2CManager {
   public:
    bool update();
    void addTransfer(uint8_t address, uint8_t send_count, uint8_t* send_data, uint8_t receive_count, uint8_t* receive_data, std::function<void()> callback);

    uint8_t readByte(uint8_t address, uint8_t subAddress);
    uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);
    uint8_t writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

   private:
    struct QueueItem {
        I2CTransfer item;
        std::unique_ptr<QueueItem> nextItem;
    };
    class TransferQueue {
       public:
        I2CTransfer& front() const;
        void push(I2CTransfer&& newItem);
        void pop();
        bool empty() const;

       private:
        std::unique_ptr<QueueItem> firstItem{nullptr};
    };
    TransferQueue transfers;
    bool waiting_for_data{false};

};  // class I2CManager

I2CManager& i2c();

#endif
