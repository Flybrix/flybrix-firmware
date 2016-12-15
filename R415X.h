/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <R415X.h/cpp>

    Uses a timer to receive data from the Orange R415X receiver module.

*/

#ifndef R415X_h
#define R415X_h

#include "Arduino.h"

class State;

class PPMchannel {
   public:
    PPMchannel(){};

    uint16_t val = 1500;

    uint16_t mid = 1500;
    uint16_t deadzone = 0;
    static const uint16_t min = 1100;
    static const uint16_t max = 1900;

    uint16_t applyDeadzone() {
        if (val > mid + deadzone) {
            return val - deadzone;
        }
        if (val < mid - deadzone) {
            return val + deadzone;
        }
        return mid;
    }

    bool isLow() {
        return ((val - min) < (max - min) / 10);
    };
    bool isHigh() {
        return ((max - val) < (max - min) / 10);
    };
    bool isMid() {
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

class R415X {
   public:
    R415X();
    void getCommandData(State* state);

    struct __attribute__((packed)) ChannelProperties {
        ChannelProperties();
        bool verify() const;
        uint8_t assignment[6];
        uint8_t inversion;     // bitfield order is {throttle_channel, pitch_channel, roll_channel, yaw_channel, x, x, x, x} (LSB-->MSB)
        uint16_t midpoint[6];  // ideally 1500usec
        uint16_t deadzone[6];  // usec units
    } channel;

    static_assert(sizeof(ChannelProperties) == 6 + 1 + 6 * 2 * 2, "Data is not packed");

   private:
    PPMchannel throttle;
    PPMchannel pitch;
    PPMchannel roll;
    PPMchannel yaw;
    PPMchannel AUX1;
    PPMchannel AUX2;

};  // class R415X

// global variables used by the interrupt callback
#define RC_CHANNEL_COUNT 6
extern volatile uint16_t RX[RC_CHANNEL_COUNT];         // filled by the interrupt with valid data
extern volatile uint16_t RX_errors;                    // count dropped frames
extern volatile uint16_t startPulse;                   // keeps track of the last received pulse position
extern volatile uint16_t RX_buffer[RC_CHANNEL_COUNT];  // buffer data in anticipation of a valid frame
extern volatile uint8_t RX_channel;                    // we are collecting data for this channel

#define RX_PPM_SYNCPULSE_MIN 7500   // 2.5ms
#define RX_PPM_SYNCPULSE_MAX 48000  // 16 ms (seems to be about 13.4ms on the scope)

#endif
