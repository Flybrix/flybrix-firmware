/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com

    Uses a timer to receive data from a 6 channel cPPM output.
*/

#ifndef receiver_h
#define receiver_h

#include "Arduino.h"
#include "utility/rcHelpers.h"

class Receiver;

class PPMchannel {
   public:
    PPMchannel(){};

    uint16_t val = 1500;

    uint16_t mid = 1500;
    uint16_t deadzone = 0;

    bool inverted = false;

    static constexpr uint16_t min = 1100;
    static constexpr uint16_t max = 1900;
    static constexpr uint16_t range = max - min;
    static constexpr uint16_t threshold = range / 10; // 10% threshold

    inline uint16_t absolute() const {
        return val - min;
    }

    inline uint16_t absoluteWithThreshold() const {
        return val - (min + threshold);
    }

    inline uint16_t offset() const {
        return val - mid;
    }

    template<typename T>
    inline T applyInversion(T value) const {
        return inverted ? -value : value;
    }

    inline uint16_t decodeAbsolute() const {
        return constrain(absolute() * 4095 / range, 0, 4095);
    }

    inline uint16_t decodeAbsoluteWithThreshold() const {
        return constrain(absoluteWithThreshold() * 4095 / (range - threshold), 0, 4095);
    }

    inline uint16_t decodeOffset() const {
        return constrain(applyInversion(offset()) * 4095 / range, -2047, 2047);
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

class Receiver {
   public:
    Receiver();
    RcState query();

    static constexpr uint8_t recovery_rate{1};
    static constexpr uint8_t refresh_delay_tolerance{1};

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
    // Detects in-flight failure by noticing a pattern that is impossible to perform by hand:
    // * RC sending both an arming AUX and a nonzero throttle
    // * Next frame we send disarm and a zero throttle
    // Perfect frame timing would need the operator to intentionally time things down to 10ms
    // Since that will not happen while normally operating the device, we know that this
    // happens only when receiver switches to default
    class ErrorTracker final {
       public:
        bool check(const RcCommand&);
        void reportFailure();

       private:
        bool was_legal_{false};
        bool was_flying_{false};
    } error_tracker_;

    void updateChannels();
    void updateChannel(std::size_t idx, PPMchannel& target);

    PPMchannel throttle;
    PPMchannel pitch;
    PPMchannel roll;
    PPMchannel yaw;
    PPMchannel AUX1;
    PPMchannel AUX2;

};  // class Receiver

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
