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
#include "cobs.h"

class String;
using CobsReaderBuffer = CobsReader<500>;

class Logger {
   public:
    explicit Logger(const char* base_name);

    void write(const uint8_t* data, size_t length);

   private:
    char filename[64]{'\0'};
};

class Messenger {
   public:
    explicit Messenger(const char* base_name);

    bool read();
    CobsReaderBuffer& buffer();

   private:
    char filename[64]{'\0'};
    size_t cursor{0};
    CobsReaderBuffer data_input;
};

#endif
