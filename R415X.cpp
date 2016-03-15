/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "R415X.h"
#include "board.h"

volatile uint16_t RX[RC_CHANNEL_COUNT];  // filled by the interrupt with valid data
volatile uint16_t RX_errors = 0;  // count dropped frames
volatile uint16_t startPulse = 0;  // keeps track of the last received pulse position
volatile uint16_t RX_buffer[RC_CHANNEL_COUNT];  // buffer data in anticipation of a valid frame
volatile uint8_t RX_channel = 0;  // we are collecting data for this channel

R415X::R415X() {
    attemptToBind(50);
    initialize_isr();
}

void R415X::initialize_isr(void) {
    pinMode(board::RX_DAT, INPUT);  // WE ARE ASSUMING RX_DAT IS PIN 3 IN FTM1 SETUP!

    for (uint8_t i = 0; i <= RC_CHANNEL_COUNT; i++) {
        RX[i] = 1100;
    }

    // FLEX Timer1 input filter configuration
    // 4+4*val clock cycles, 48MHz = 4+4*7 = 32 clock cycles = 0.75us
    FTM1_FILTER = 0x07;
    // FLEX Timer1 configuration
    SIM_SCGC6 |= SIM_SCGC6_FTM1;  // Enable FTM1 clock
    FTM1_SC = 0x0C;               // TOF=0 TOIE=0 CPWMS=0 CLKS=01 (system clock) PS=100 (divide by 16)
    FTM1_MOD = 0xFFFF;            // modulo to max
    FTM1_C0SC = 0x44;             // CHF=0 CHIE=1 MSB=0 MSA=0 ELSB=0 ELSA=1 DMA=0
    // Enable FTM1 interrupt inside NVIC
    NVIC_ENABLE_IRQ(IRQ_FTM1);
    // PIN configuration, alternative function 3
    PORTA_PCR12 |= 0x300;  // Signal sampling is being done via PORTA_PCR12 (PIN 3).
}

extern "C" void ftm1_isr(void) {
    // save current interrupt count/time
    uint16_t stopPulse = FTM1_C0V;

    // clear channel interrupt flag (CHF)
    FTM1_C0SC &= ~0x80;

    uint16_t pulseWidth = stopPulse - startPulse;

    // Error / Sanity check
    // too short to be data  : pulseWidth < 900us
    // between data and sync : (pulseWidth > 2100us and pulseWidth < RX_PPM_SYNCPULSE_MIN)
    // too long : pulseWidth > RX_PPM_SYNCPULSE_MAX;
    if (pulseWidth < 2700 || (pulseWidth > 6300 && pulseWidth < RX_PPM_SYNCPULSE_MIN) || pulseWidth > RX_PPM_SYNCPULSE_MAX) {
        RX_errors++;

        RX_channel = RC_CHANNEL_COUNT + 1;           // set RX_channel out of range to drop frame
    } else if (pulseWidth > RX_PPM_SYNCPULSE_MIN) {  // this is the sync pulse
        if (RX_channel <= RC_CHANNEL_COUNT) {        // valid frame = push from our buffer
            for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
                RX[i] = RX_buffer[i];
            }
        }
        // restart the channel counter whether the frame was valid or not
        RX_channel = 0;
    } else {                                         // this pulse is channel data
        if (RX_channel < RC_CHANNEL_COUNT) {         // extra channels will get ignored here
            RX_buffer[RX_channel] = pulseWidth / 3;  // Store measured pulse length in usec
            RX_channel++;                            // Advance to next channel
        }
    }
    startPulse = stopPulse;  // Save time at pulse start
}

void R415X::attemptToBind(uint16_t milliseconds) {
    pinMode(board::RX_DAT, OUTPUT);
    digitalWrite(board::RX_DAT, LOW);
    delay(milliseconds);
    pinMode(board::RX_DAT, INPUT);  // WE ARE ASSUMING RX_DAT IS PIN 3 IN FTM1 SETUP!

    // after we bind, we must setup our timer again.
    initialize_isr();
}
