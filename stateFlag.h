#ifndef STATE_FLAG_H
#define STATE_FLAG_H

#include <cstdint>

using FlagData = uint16_t;

enum Status : FlagData {
    BOOT = 0x0001,
    MPU_FAIL = 0x0002,
    BMP_FAIL = 0x0004,
    RX_FAIL = 0x0008,

    IDLE = 0x0010,

    ENABLING = 0x0020,
    CLEAR_MPU_BIAS = 0x0040,
    SET_MPU_BIAS = 0x0080,

    FAIL_STABILITY = 0x0100,
    FAIL_ANGLE = 0x0200,
    FAIL_OTHER = 0x4000,

    ENABLED = 0x0400,

    BATTERY_LOW = 0x0800,
    TEMP_WARNING = 0x1000,
    LOG_FULL = 0x2000,

    OVERRIDE = 0x8000,
};

class StateFlag final {
   public:
    void set(FlagData bits);
    void clear(FlagData bits);
    bool is(FlagData bits) const;
    FlagData value() const;

   private:
    FlagData flag_{0};
};

#endif /* STATE_FLAG_H */
