/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
    *
    *
*/

#include "i2cManager.h"
#include <i2c_t3.h>

void I2CManager::addTransfer(uint8_t address, uint8_t send_count, uint8_t *send_data, uint8_t receive_count, uint8_t *receive_data, CallbackProcessor *cb_object) {
    I2CTransfer *new_transfer = (I2CTransfer *)malloc(sizeof(I2CTransfer));
    new_transfer->address = address;
    new_transfer->send_count = send_count;
    new_transfer->send_data = send_data;
    new_transfer->receive_count = receive_count;
    new_transfer->receive_data = receive_data;
    new_transfer->cb_object = cb_object;
    new_transfer->next = NULL;

    if (!transfer) {
        transfer = new_transfer;
    } else {
        I2CTransfer *last = transfer;
        while (last->next) {
            last = last->next;
        }
        last->next = new_transfer;
    }
}

void I2CManager::update() {
    if (waiting_for_data) {
        if (Wire.available() == transfer->receive_count) {
            for (uint8_t i = 0; i < transfer->receive_count; i++) {
                transfer->receive_data[i] = Wire.read();
            }
            transfer->cb_object->processCallback(transfer->receive_count, transfer->receive_data);

            I2CTransfer *completed_transfer = transfer;
            if (transfer->next) {
                transfer = transfer->next;
                free(completed_transfer);
            } else {
                free(transfer);
                transfer = NULL;
            }
            waiting_for_data = false;
        }
    } else if (transfer && !waiting_for_data) {
        // begin new transfer
        Wire.beginTransmission(transfer->address);
        Wire.write(transfer->send_data, transfer->send_count);
        uint8_t error = Wire.endTransmission();
        if (error == 0 && transfer->receive_count > 0) {
            waiting_for_data = true;
            Wire.requestFrom(transfer->address, transfer->receive_count);
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

uint8_t I2CManager::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest) {
    int error;

#ifdef BMP280_SERIAL_DEBUG
    Serial.print("attempting read register 0x");
    Serial.print(subAddress, HEX);
    Serial.print(" from address 0x");
    Serial.println(address, HEX);
#endif

    Wire.beginTransmission(address);
    Wire.write(subAddress);
    error = Wire.endTransmission();
    if (error == 0) {
        Wire.requestFrom(address, count);
        while (Wire.available() != count)
            ;  // wait until bytes are ready
        for (int i = 0; i < count; i++) {
            dest[i] = Wire.read();

#ifdef BMP280_SERIAL_DEBUG
            Serial.print("received 0x");
            Serial.println(dest[i], HEX);
#endif
        }

        return (1);
    }
    return (0);
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
