/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef STATE_FLAG_H
#define STATE_FLAG_H

#include <cstdint>

using FlagData = uint16_t;

enum Status : FlagData {
    NO_SIGNAL = 1 << 3,
    IDLE = 1 << 4,
    ARMING = 1 << 5,
    RECORDING_SD = 1 << 6,
    LOOP_SLOW = 1 << 8,
    ARMED = 1 << 10,
    BATTERY_LOW = 1 << 11,
    BATTERY_CRITICAL = 1 << 12,
    LOG_FULL = 1 << 13,
    CRASH_DETECTED = 1 << 14,
    OVERRIDE = 1 << 15,
};

class StateFlag final {
   public:
    void set(FlagData bits);
    void clear(FlagData bits);
    void assign(FlagData bits, bool value);
    bool is(FlagData bits) const;
    FlagData value() const;

   private:
    FlagData flag_{0};
};

#endif /* STATE_FLAG_H */
