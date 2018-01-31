/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "version.h"

#include "debug.h"
#include "led.h"

bool Version::verify() const {
    if (major == FIRMWARE_VERSION_A && minor == FIRMWARE_VERSION_B && patch == FIRMWARE_VERSION_C) {
        return true;
    }
    DebugPrint("Configuration versions do not match");
    return false;
}

uint32_t encode(uint32_t color) {
    bool lighter{color & 8};
    uint32_t strength{lighter ? 0xcc : 0x11};
    uint32_t base{lighter ? 0x010101 : 0x000000};
    return (((color & 4) >> 2) + ((color & 2) << 7) + ((color & 1) << 16)) * strength + base;
}

void Version::display(LED& led) const {
    led.set(LED::SOLID, encode(minor), 0, encode(major), encode(patch), false, false);
    led.update();
    led.set(LED::NO_OVERRIDE, 0, false, false);
}
