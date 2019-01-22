/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "ClockTime.h"

#include <Arduino.h>

ClockTime::ClockTime() : ClockTime(micros()) {};

ClockTime::ClockTime(uint32_t value) : value_{value} {}

ClockTime ClockTime::now() {
    return ClockTime(micros());
}

ClockTime ClockTime::zero() {
    return ClockTime(0);
}

bool ClockTime::isNotReasonable(uint32_t delta) {
    return delta == 0 || (delta > (1 << 30));
}

uint32_t ClockTime::readClockTick() const {
    return value_;
}

uint32_t ClockTime::operator-(const ClockTime& v) {
    return value_ - v.value_;
}

uint32_t ClockTime::operator-(ClockTime&& v) {
    return value_ - v.value_;
}
