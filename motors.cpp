/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "motors.h"
#include "board.h"

Motors::Motors() {
    // REFERENCE: https://www.pjrc.com/teensy/td_pulse.html
    analogWriteResolution(12);  // actual resolution depends on frequency

    for (auto pin : board::PWM)
        pinMode(pin, OUTPUT);

    for (auto ftm : board::FTM)  // changes all pins on FTM2 and FTM0
        analogWriteFrequency(ftm, 11718);
}

void Motors::updateAllChannels(bool enabled) {
    for (uint8_t motor = 0; motor < 8; motor++) {
        // 12 bit output
        output_[motor] = constrain(output_[motor], 0, 4095);
        analogWrite(board::PWM[motor], enabled ? output_[motor] : 0);
    }
}
