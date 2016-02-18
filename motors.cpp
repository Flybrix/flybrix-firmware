/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "motors.h"
#include "state.h"

Motors::Motors(State* __state) {
    state = __state;

    // REFERENCE: https://www.pjrc.com/teensy/td_pulse.html
    analogWriteResolution(12);  // actual resolution depends on frequency

    // FTM2
    pinMode(PWM0, OUTPUT);
    pinMode(PWM1, OUTPUT);
    analogWriteFrequency(PWM0, 11718);  // changes all pins on FTM2

    // FTM0
    pinMode(PWM2, OUTPUT);
    pinMode(PWM3, OUTPUT);
    pinMode(PWM4, OUTPUT);
    pinMode(PWM5, OUTPUT);
    pinMode(PWM6, OUTPUT);
    pinMode(PWM7, OUTPUT);
    analogWriteFrequency(PWM2, 11718);  // changes all pins on FTM0
}

void Motors::updateAllChannels() {
    // 12 bit output

    for (uint8_t motor = 0; motor < 8; motor++) {
        state->MotorOut[motor] = constrain(state->MotorOut[motor], 0, 4095);
    }

    if (state->is(STATUS_ENABLED) || state->is(STATUS_OVERRIDE)) {
        analogWrite(PWM0, state->MotorOut[0]);
        analogWrite(PWM1, state->MotorOut[1]);
        analogWrite(PWM2, state->MotorOut[2]);
        analogWrite(PWM3, state->MotorOut[3]);
        analogWrite(PWM4, state->MotorOut[4]);
        analogWrite(PWM5, state->MotorOut[5]);
        analogWrite(PWM6, state->MotorOut[6]);
        analogWrite(PWM7, state->MotorOut[7]);
    }
	else
	{
		analogWrite(PWM0, 0);
        analogWrite(PWM1, 0);
        analogWrite(PWM2, 0);
        analogWrite(PWM3, 0);
        analogWrite(PWM4, 0);
        analogWrite(PWM5, 0);
        analogWrite(PWM6, 0);
        analogWrite(PWM7, 0);
    
	}
}
