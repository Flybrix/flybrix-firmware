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

// Having a missing card causes great latency at the first attempt of access
// We can use this command to call it prior to the flight starting
void startupCard();

void writeToCard(const uint8_t* data, size_t length);

void commitWriteToCard();

#endif
