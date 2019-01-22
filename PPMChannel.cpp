/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "PPMChannel.h"
#include "Debug.h"

namespace {
    template<typename T>
    inline static T clampUnipolar(const T value) {
        return constrain(value, 0, PPMChannel::max_unipolar_output);
    }

    template<typename T>
    inline static T clampBipolar(const T value) {
        return constrain(value, -PPMChannel::max_bipolar_output, PPMChannel::max_bipolar_output);
    }
}

bool PPMChannel::validateDeadzoneAndMidpoint(uint16_t deadzone, uint16_t midpoint, bool log) {
    bool validMidpoint = PPMChannel::isValidMidpoint(midpoint);
    bool validDeadzone = PPMChannel::isValidDeadzoneForMidpoint(deadzone, midpoint);

    if (log) {
        if (!validMidpoint) {
            DebugPrint("Channel midpoints must be within the channel range");
        }
        if (!validDeadzone) {
            DebugPrint("Channel deadzone cannot be larger than the midpoint value and range");
        }
    }

    return validMidpoint && validDeadzone;
}

bool PPMChannel::isValidMidpoint(uint16_t midpoint) {
    return midpoint >= PPMChannel::min && midpoint <= PPMChannel::max;
}

bool PPMChannel::isValidDeadzoneForMidpoint(uint16_t deadzone, uint16_t midpoint) {
    return deadzone < range && deadzone < midpoint;
}

void PPMChannel::trimDeadzone() {
    if (val > mid + deadzone) {
        val -= deadzone;
    } else if (val < mid - deadzone) {
        val += deadzone;
    }
}

int16_t PPMChannel::decodeAbsolute() const {
    return clampUnipolar(rawAbsolute() * max_unipolar_output / range);
}

int16_t PPMChannel::decodeAbsoluteWithThreshold() const {
    return clampUnipolar(rawAbsoluteWithThreshold() * max_unipolar_output / (range - threshold));
}

int16_t PPMChannel::decodeOffset() const {
    int16_t rawOffset = rawMidOffset();
    if (inverted) {
        rawOffset = -rawOffset;
    }
    return clampBipolar(rawOffset * max_unipolar_output / range);
}

bool PPMChannel::isLow() const {
    return (val - min) < (max - min) / 10;
};

bool PPMChannel::isMid() const {
    return abs(val - mid) < (max - min) / 10;
};

bool PPMChannel::isHigh() const {
    return (max - val) < (max - min) / 10;
};

inline int16_t PPMChannel::rawAbsolute() const {
    return val - min;
}

inline int16_t PPMChannel::rawAbsoluteWithThreshold() const {
    return val - (min + threshold);
}

inline int16_t PPMChannel::rawMidOffset() const {
    return val - mid;
}
