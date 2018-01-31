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

CobsReaderBuffer* readSerial();
void writeSerial(uint8_t* data, size_t length);
uint32_t flushSerial(uint32_t times);
void setBluetoothUart(const DeviceName& name);
#endif
