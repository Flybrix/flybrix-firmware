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
// Called for each complete message the moment its last byte arrives.
using MessageHandler = void (*)(CobsReaderBuffer& message);

// Reads all waiting bytes. The receive buffer holds a single message and the next byte starts overwriting it,
// so pass a handler to process each message as soon as it completes (otherwise messages can be dropped).
bool bluetooth_getData(MessageHandler on_message = nullptr);

CobsReaderBuffer* bluetooth_readData();

void printSerialReport();
#endif




