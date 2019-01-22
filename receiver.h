/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com

    Uses a timer to receive data from a 6 channel cPPM output.
*/

#ifndef receiver_h
#define receiver_h

#include "Arduino.h"
#include "RcMux.h"
#include "PPMChannel.h"

constexpr auto RC_CHANNEL_COUNT = 6;

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

    uint16_t ppm[RC_CHANNEL_COUNT];

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
    void updateChannel(std::size_t idx, PPMChannel& target);

    PPMChannel throttle;
    PPMChannel pitch;
    PPMChannel roll;
    PPMChannel yaw;
    PPMChannel AUX1;
    PPMChannel AUX2;

};  // class Receiver

#endif
