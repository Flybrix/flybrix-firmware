/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "motors.h"
#include "board.h"
#include "state.h"

Motors::Motors(State* __state) {
    state = __state;

    // REFERENCE: https://www.pjrc.com/teensy/td_pulse.html
    analogWriteResolution(12);  // actual resolution depends on frequency

    // FTM2
    pinMode(board::PWM[0], OUTPUT);
    pinMode(board::PWM[1], OUTPUT);
    analogWriteFrequency(board::PWM[0], 11718);  // changes all pins on FTM2

    // FTM0
    pinMode(board::PWM[2], OUTPUT);
    pinMode(board::PWM[3], OUTPUT);
    pinMode(board::PWM[4], OUTPUT);
    pinMode(board::PWM[5], OUTPUT);
    pinMode(board::PWM[6], OUTPUT);
    pinMode(board::PWM[7], OUTPUT);
    analogWriteFrequency(board::PWM[2], 11718);  // changes all pins on FTM0
}

void Motors::updateAllChannels() {
    // 12 bit output

    for (uint8_t motor = 0; motor < 8; motor++) {
        state->MotorOut[motor] = constrain(state->MotorOut[motor], 0, 4095);
    }

    if (state->is(STATUS_ENABLED) || state->is(STATUS_OVERRIDE)) {
        analogWrite(board::PWM[0], state->MotorOut[0]);
        analogWrite(board::PWM[1], state->MotorOut[1]);
        analogWrite(board::PWM[2], state->MotorOut[2]);
        analogWrite(board::PWM[3], state->MotorOut[3]);
        analogWrite(board::PWM[4], state->MotorOut[4]);
        analogWrite(board::PWM[5], state->MotorOut[5]);
        analogWrite(board::PWM[6], state->MotorOut[6]);
        analogWrite(board::PWM[7], state->MotorOut[7]);
    }
	else
	{
		analogWrite(board::PWM[0], 0);
        analogWrite(board::PWM[1], 0);
        analogWrite(board::PWM[2], 0);
        analogWrite(board::PWM[3], 0);
        analogWrite(board::PWM[4], 0);
        analogWrite(board::PWM[5], 0);
        analogWrite(board::PWM[6], 0);
        analogWrite(board::PWM[7], 0);

	}
}
