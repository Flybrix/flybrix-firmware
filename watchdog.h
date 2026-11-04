/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <cstdint>

// Hardware watchdog (Kinetis WDOG on the Teensy 3.2). If the main loop stops feeding it -- a hung bus, an
// infinite loop -- the MCU resets, which stops the motors, instead of the motors holding their last PWM value.
namespace watchdog {
constexpr uint32_t TIMEOUT_MSEC = 250;

void begin();    // enable; call at the end of setup()
void feed();     // call from the main loop
void suspend();  // pause during deliberate long blocking work (see loops::Stopper)
void resume();
}  // namespace watchdog

#endif  // WATCHDOG_H
