/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <serialFork.h/cpp>

    Interface for forking serial communication channels.
*/

#ifndef SERIAL_FORK_H
#define SERIAL_FORK_H

#include <cstdint>
#include "cobs.h"

class DeviceName;

CobsReaderBuffer* readSerial();
void writeSerial(uint8_t* data, size_t length);
void flushSerial();
void setBluetoothUart(const DeviceName& name);
#endif
