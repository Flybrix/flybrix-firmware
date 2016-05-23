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
// Card startup takes a long time to perform
void startup();

// Opening and clearing the file takes a long time to perform
void openFile();

void write(const uint8_t* data, size_t length);

// File closing (saving and truncating the file) takes a long time to perform
void closeFile();
}

#endif
