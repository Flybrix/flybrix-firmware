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
#include <i2c_t3.h>

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

    V0_DETECT = A13,  // ADC0_DM3
    I0_DETECT = A10,  // ADC0_DP0
    I1_DETECT = A11,  // ADC0_DM0

    RX_DAT = 3,  // 28  --- MUST BE PIN 3

    MPU_INTERRUPT = 17,  // 36
};

constexpr i2c_pins I2C_PINS{I2C_PINS_18_19};
constexpr i2c_pullup I2C_PULLUP{I2C_PULLUP_EXT};

constexpr uint8_t PWM[]{
    25,  // 42
    32,  // 41
    22,  // 44
    20,  // 62
    23,  // 45
    21,  // 63
    9,   // 46
    5,   // 64
};

constexpr uint8_t FTM[]{
    // TODO: properly consider right FTM pins
    25,  // 42 | PWM[0]
    22,  // 44 | PWM[2]
};
}
}

#endif
