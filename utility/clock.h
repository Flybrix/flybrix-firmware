/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef clock_h
#define clock_h

#include <Arduino.h>

class __attribute__((packed)) ClockTime final {
   public:
    ClockTime() : ClockTime(micros()){};

    inline static ClockTime now() {
        return ClockTime(micros());
    }

    inline static ClockTime zero() {
        return ClockTime(0);
    }

    inline static bool isNotReasonable(uint32_t delta) {
        return delta == 0 || (delta > (1 << 30));
    }

    uint32_t readClockTick() {
        return value_;
    }

    uint32_t operator-(const ClockTime& v) {
        return value_ - v.value_;
    }

    uint32_t operator-(ClockTime&& v) {
        return value_ - v.value_;
    }

   private:
    explicit ClockTime(uint32_t value) : value_{value} {
    }

    uint32_t value_{0};
};

static_assert(sizeof(ClockTime) == 4, "ClockTime data is not packed");

#endif  // clock_h
