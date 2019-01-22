/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef ppm_channel_h
#define ppm_channel_h

#include <Arduino.h>

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

class PPMChannel {
public:
    static bool validateDeadzoneAndMidpoint(uint16_t deadzone, uint16_t midpoint, bool log = true);

    static constexpr uint16_t min = 1100;
    static constexpr uint16_t max = 1900;
    static constexpr uint16_t range = max - min;
    static constexpr uint16_t threshold = range / 10; // 10% threshold
    static constexpr int16_t max_unipolar_output = 4095;
    static constexpr int16_t max_bipolar_output = 2047;

    void trimDeadzone();

    int16_t decodeAbsolute() const;
    int16_t decodeAbsoluteWithThreshold() const;
    int16_t decodeOffset() const;

    bool isLow() const;
    bool isMid() const;
    bool isHigh() const;

    uint16_t val = 1500;
    uint16_t mid = 1500;
    uint16_t deadzone = 0;
    bool inverted = false;

private:
    static bool isValidMidpoint(uint16_t midpoint);
    static bool isValidDeadzoneForMidpoint(uint16_t deadzone, uint16_t midpoint);

    inline int16_t rawAbsolute() const;
    inline int16_t rawAbsoluteWithThreshold() const;
    inline int16_t rawMidOffset() const;
};

#endif //ppm_channel_h
