/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef CLOCK_TIME_H
#define CLOCK_TIME_H

#include <cstdint>

class __attribute__((packed)) ClockTime final {
public:
    ClockTime();
    static ClockTime now();
    static ClockTime zero();
    static bool isNotReasonable(uint32_t delta);

    uint32_t readClockTick() const;
    uint32_t operator-(const ClockTime& v);
    uint32_t operator-(ClockTime&& v);

private:
    explicit ClockTime(uint32_t value);

    uint32_t value_{0};
};

static_assert(sizeof(ClockTime) == 4, "ClockTime data is not packed");

#endif // CLOCK_TIME_H
