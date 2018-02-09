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
    NO_SIGNAL = 0x0008,

    IDLE = 0x0010,

    ARMING = 0x0020,
    ARMED = 0x0400,

    BATTERY_LOW = 0x0800,
    LOG_FULL = 0x2000,

    OVERRIDE = 0x8000,
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
