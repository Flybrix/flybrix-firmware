/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <cardManagement.h/cpp>

    General interaction with the SD card, in the forms of logging or reading.
*/

#ifndef CARD_MANAGEMENT_H
#define CARD_MANAGEMENT_H

#include <Arduino.h>

namespace sdcard {

enum class State {
    Closed,
    WriteStates,
    WriteCommands,
    ReadCommands,
};

// Card startup takes a long time to perform
//
// Any SD card operation requires establishing a connection to it.
// Establishing that connection causes lag.
// Run this at boot to prevent lag later on.
void startup();

State getState();

namespace writing {
// Opening and clearing the file takes a long time to perform
void open();
// File closing (saving and truncating the file) takes a long time to perform
void close();

void write(const uint8_t* data, size_t length);

// We can set a lock to SD opening/closing controls, which can only be overriden with serial commands
void setLock(bool enable);
bool isLocked();
}

namespace reading {
void open();
void close();
bool hasMore();
char read();
}
}

#endif
