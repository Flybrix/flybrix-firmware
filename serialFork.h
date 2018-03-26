/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef SERIAL_FORK_H
#define SERIAL_FORK_H

#include <cstdint>
#include "cobs.h"

class DeviceName;

void writeSerial(uint8_t* data, size_t length);
void setBluetoothUart(const DeviceName& name);

bool usb_sendData();
bool usb_getData();
bool bluetooth_sendData();
bool bluetooth_getData();

CobsReaderBuffer* usb_readData();
CobsReaderBuffer* bluetooth_readData();

void printSerialReport();
#endif




