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

class R415X {
   public:
    R415X();
    void attemptToBind(uint16_t milliseconds);

   private:
    void initialize_isr(void);

};  // class R415X

// global variables used by the interrupt callback
#define RC_CHANNEL_COUNT 6
extern volatile uint16_t RX[RC_CHANNEL_COUNT];  // filled by the interrupt with valid data
extern volatile uint16_t RX_errors;  // count dropped frames
extern volatile uint16_t startPulse;  // keeps track of the last received pulse position
extern volatile uint16_t RX_buffer[RC_CHANNEL_COUNT];  // buffer data in anticipation of a valid frame
extern volatile uint8_t RX_channel;  // we are collecting data for this channel

#define RX_PPM_SYNCPULSE_MIN 7500   // 2.5ms
#define RX_PPM_SYNCPULSE_MAX 48000  // 16 ms (seems to be about 13.4ms on the scope)

#endif
