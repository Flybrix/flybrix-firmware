/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef RC_COMMAND_H
#define RC_COMMAND_H

#include <cstdint>

struct RcCommand final {
    using Axis = int16_t;
    // bitfield order is {AUX1_low, AUX1_mid, AUX1_high, AUX2_low, AUX2_mid, AUX2_high, x, x} (LSB-->MSB)
    using AuxMask = uint8_t;

    enum class AUX : AuxMask {
        None = 0,
        Low = 1,
        Mid = 2,
        High = 4,
    };

    AuxMask auxMask() const;

    AUX aux1() const;
    AUX aux2() const;
    Axis throttle() const;
    Axis pitch() const;
    Axis roll() const;
    Axis yaw() const;

    void resetAxes();
    void setThrottle(Axis value);
    void setPitch(Axis value);
    void setRoll(Axis value);
    void setYaw(Axis value);

    void parseBools(bool l1, bool m1, bool h1, bool l2, bool m2, bool h2);
    void parseAuxMask(AuxMask auxmask);

private:
    static AUX parseSingleBools(bool low, bool mid, bool high);

    AUX aux1_{AUX::None};
    AUX aux2_{AUX::None};
    Axis throttle_{0};
    Axis pitch_{0};
    Axis roll_{0};
    Axis yaw_{0};
};

#endif // RC_COMMAND_H
