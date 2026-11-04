/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "watchdog.h"

#include <Arduino.h>

// Teensyduino's startup code leaves WDOG_STCTRLH_ALLOWUPDATE set, which is what allows configuring it here.
// Every configuration write must follow the unlock sequence within a short window, so interrupts are disabled.

namespace watchdog {
namespace {
bool started{false};
bool suspended{false};

inline void unlock() {
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    __asm__ volatile("nop");
    __asm__ volatile("nop");
}

void configure(bool enable) {
    noInterrupts();
    unlock();
    if (enable) {
        // 1kHz LPO clock, no prescaler -> the timeout value is in milliseconds
        WDOG_TOVALH = (uint16_t)(TIMEOUT_MSEC >> 16);
        WDOG_TOVALL = (uint16_t)(TIMEOUT_MSEC & 0xFFFF);
        WDOG_PRESC = 0;
        WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN;
    } else {
        WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;
    }
    interrupts();
}

inline void refresh() {
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}
}  // namespace

void begin() {
    started = true;
    suspended = false;
    configure(true);
    refresh();
}

void feed() {
    if (!started || suspended) {
        return;
    }
    // refreshing on every loop pass is wasteful; once a millisecond is plenty for a 250ms timeout
    static uint32_t last_feed_usec{0};
    uint32_t now = micros();
    if (now - last_feed_usec < 1000) {
        return;
    }
    last_feed_usec = now;
    refresh();
}

void suspend() {
    if (!started || suspended) {
        return;
    }
    suspended = true;
    configure(false);
}

void resume() {
    if (!started || !suspended) {
        return;
    }
    suspended = false;
    configure(true);
    refresh();
}
}  // namespace watchdog
