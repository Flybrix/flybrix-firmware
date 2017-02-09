/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <version.h/cpp>

    Definitions for the firmware version.

*/

#ifndef version_h
#define version_h

#include "debug.h"

#define FIRMWARE_VERSION_A 1
#define FIRMWARE_VERSION_B 5
#define FIRMWARE_VERSION_C 0

class LED;

struct __attribute__((packed)) Version {
    Version() : major(FIRMWARE_VERSION_A), minor(FIRMWARE_VERSION_B), patch(FIRMWARE_VERSION_C) {
    }
    void display(LED& led) const;
    bool verify() const;
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
};

static_assert(sizeof(Version) == 3, "Data is not packed");

#endif
