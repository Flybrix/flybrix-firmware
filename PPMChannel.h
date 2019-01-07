/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef ppm_channel_h
#define ppm_channel_h

#include "Arduino.h"

class PPMchannel {
private:
    static constexpr uint16_t min = 1100;
    static constexpr uint16_t max = 1900;
    static constexpr uint16_t range = max - min;
    static constexpr uint16_t threshold = range / 10; // 10% threshold
    static constexpr uint16_t max_unipolar_output = 4095;
    static constexpr uint16_t max_bipolar_output = 2047;

    template<typename T>
    inline static T clampUnipolar(const T value) {
        return constrain(value, 0, max_unipolar_output);
    }

    template<typename T>
    inline static T clampBipolar(const T value) {
        return constrain(value, -max_bipolar_output, max_bipolar_output);
    }

public:
    PPMchannel() {};

    uint16_t val = 1500;

    uint16_t mid = 1500;
    uint16_t deadzone = 0;

    bool inverted = false;

    static bool isValidMidpoint(uint16_t midpoint) {
        return midpoint >= PPMchannel::min && midpoint <= PPMchannel::max;
    }

    static bool isValidDeadzoneForMidpoint(uint16_t deadzone, uint16_t midpoint) {
        return deadzone < range && deadzone < midpoint;
    }

    static bool validateDeadzoneAndMidpoint(uint16_t deadzone, uint16_t midpoint, bool log = true);

    inline uint16_t rawAbsolute() const {
        return val - min;
    }

    inline uint16_t rawAbsoluteWithThreshold() const {
        return val - (min + threshold);
    }

    inline uint16_t rawMidOffset() const {
        return val - mid;
    }

    template<typename T>
    inline T applyInversion(T value) const {
        return inverted ? -value : value;
    }

    inline uint16_t decodeAbsolute() const {
        return clampUnipolar(rawAbsolute() * max_unipolar_output / range);
    }

    inline uint16_t decodeAbsoluteWithThreshold() const {
        return clampUnipolar(rawAbsoluteWithThreshold() * max_unipolar_output / (range - threshold));
    }

    inline uint16_t decodeOffset() const {
        return clampBipolar(applyInversion(rawMidOffset()) * max_unipolar_output / range);
    }

    uint16_t applyDeadzone() const {
        if (val > mid + deadzone) {
            return val - deadzone;
        }
        if (val < mid - deadzone) {
            return val + deadzone;
        }
        return mid;
    }

    void trimDeadzone() {
        val = applyDeadzone();
    }

    bool isLow() const {
        return ((val - min) < (max - min) / 10);
    };

    bool isHigh() const {
        return ((max - val) < (max - min) / 10);
    };

    bool isMid() const {
        return (abs(val - mid) < (max - min) / 10);
    };

    // R/C controllers are sold with a specified "mode" that is either "mode 1" or "mode 2"
    // our default settings assume a "mode 2" controller, but the channels can be remapped if desired.
    // "mode 2" puts "throttle/yaw" on left stick and "pitch/roll" on the right
    // "mode 1" puts "pitch/yaw" on left stick and "throttle/roll" on the right
    // in both modes, sticks "down" or "right" produce LOW ppm values, while sticks "up" or "left" give high values
    //
    // We want our sticks to map cleanly over to our flyer's coordinate system:
    //   ( +x = right, +y = forward, +z = up) which implies (+pitch = nose up, +roll = RHS down, +yaw = CCW viewed from above)
    //
    // ***** To match this convention, we must invert the direction of the pitch and roll commands *****
    //
    // This is done using 'CONFIG.data.channelInversion'.
    //
};

#endif //ppm_channel_h
