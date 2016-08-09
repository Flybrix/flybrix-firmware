/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <version.h/cpp>

    Definitions for the firmware version.

*/

#include "version.h"

#include "debug.h"

bool Version::verify() const {
    if (major == FIRMWARE_VERSION_A && minor == FIRMWARE_VERSION_B && patch == FIRMWARE_VERSION_C) {
        return true;
    }
    DebugPrint("Configuration versions do not match");
    return false;
}
