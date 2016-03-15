/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <board.h>

    Definitions of pins used in boards.
*/

#ifndef BOARD_H
#define BOARD_H

#define LED_B_BLU 30  // 56
#define LED_B_GRN 26  // 2
#define LED_B_RED 31  // 1

#define LED_A_BLU 11  // 51
#define LED_A_GRN 12  // 52
#define LED_A_RED 28  // 53

#define GREEN_LED 13  // 50
#define RED_LED 27    // 54


#define PWM7 5  // 64
#define PWM6 9  // 46
#define PWM5 21  // 63
#define PWM4 23  // 45
#define PWM3 20  // 62
#define PWM2 22  // 44
#define PWM1 32  // 41
#define PWM0 25  // 42


#define V0_DETECT A13  // ADC0_DM3
#define I0_DETECT A10  // ADC0_DP0
#define I1_DETECT A11  // ADC0_DM0


#define RX_DAT 3  // 28  --- MUST BE PIN 3

#endif
