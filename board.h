/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <board.h>

    Definitions of pins used in boards.
*/

#ifndef BOARD_H
#define BOARD_H

#include <Arduino.h>

namespace board {
inline namespace alpha {
enum Pins : uint8_t {
    LED_B_BLU = 30,  // 56
    LED_B_GRN = 26,  // 2
    LED_B_RED = 31,  // 1

    LED_A_BLU = 11,  // 51
    LED_A_GRN = 12,  // 52
    LED_A_RED = 28,  // 53

    GREEN_LED = 13,  // 50
    RED_LED = 27,    // 54

    PWM7 = 5,   // 64
    PWM6 = 9,   // 46
    PWM5 = 21,  // 63
    PWM4 = 23,  // 45
    PWM3 = 20,  // 62
    PWM2 = 22,  // 44
    PWM1 = 32,  // 41
    PWM0 = 25,  // 42

    V0_DETECT = A13,  // ADC0_DM3
    I0_DETECT = A10,  // ADC0_DP0
    I1_DETECT = A11,  // ADC0_DM0

    RX_DAT = 3,  // 28  --- MUST BE PIN 3
};
}
}

#endif
