/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "R415X.h"
#include "board.h"

#include "state.h"

volatile uint16_t RX[RC_CHANNEL_COUNT];  // filled by the interrupt with valid data
volatile uint16_t RX_errors = 0;  // count dropped frames
volatile uint16_t startPulse = 0;  // keeps track of the last received pulse position
volatile uint16_t RX_buffer[RC_CHANNEL_COUNT];  // buffer data in anticipation of a valid frame
volatile uint8_t RX_channel = 0;  // we are collecting data for this channel

R415X::R415X() {
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

void R415X::getCommandData(State* state) {
    
    cli();  // disable interrupts

    // if R415X is working, we should never see anything less than 900!
    for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
        if (RX[i] < 900) {
            // tell state that R415X is not ready and return
            state->command_source_mask &= ~COMMAND_READY_R415X;
            sei();  // enable interrupts
            return; // don't load bad data into state
        }
    }
    
    // read data into PPMchannel objects using receiver channels assigned from configuration
    throttle.update(RX[CONFIG.data.channelAssignment[0]]);
    pitch.update(RX[CONFIG.data.channelAssignment[1]]);
    roll.update(RX[CONFIG.data.channelAssignment[2]]);
    yaw.update(RX[CONFIG.data.channelAssignment[3]]);
    AUX1.update(RX[CONFIG.data.channelAssignment[4]]);
    AUX2.update(RX[CONFIG.data.channelAssignment[5]]);

    // update midpoints from config
    throttle.mid = CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[0]];
    pitch.mid = CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[1]];
    roll.mid = CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[2]];
    yaw.mid = CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[3]];
    AUX1.mid = CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[4]];
    AUX2.mid = CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[5]];

    sei();  // enable interrupts
    
    // tell state R415X is working and translate PPMChannel data into the four command level and aux mask
    state->command_source_mask |= COMMAND_READY_R415X;

    state->command_AUX_mask = 0x00;  // reset the AUX mode bitmask
    // bitfield order is {AUX1_low, AUX1_mid, AUX1_high, AUX2_low, AUX2_mid, AUX2_high, x, x} (LSB-->MSB)
    if (AUX1.isLow()) {
        state->command_AUX_mask |= (1 << 0);
    } else if (AUX1.isMid()) {
        state->command_AUX_mask |= (1 << 1);
    } else if (AUX1.isHigh()) {
        state->command_AUX_mask |= (1 << 2);
    }
    if (AUX2.isLow()) {
        state->command_AUX_mask |= (1 << 3);
    } else if (AUX2.isMid()) {
        state->command_AUX_mask |= (1 << 4);
    } else if (AUX2.isHigh()) {
        state->command_AUX_mask |= (1 << 5);
    }
    
    // in some cases it is impossible to get a ppm channel to be close enought to the midpoint (~1500 usec) because the controller trim is too coarse to correct a small error
    // we get around this by creating a small dead zone around the midpoint of signed channel, specified in usec units
    int16_t pitch_delta = pitch.val - CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[1]];
    int16_t  roll_delta =  roll.val - CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[2]];
    int16_t   yaw_delta =   yaw.val - CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[3]];
    pitch.val = (pitch_delta > 0) ? max(CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[1]], pitch.val - CONFIG.data.channelDeadzone[CONFIG.data.channelAssignment[1]]) : 
                                    min(CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[1]], pitch.val + CONFIG.data.channelDeadzone[CONFIG.data.channelAssignment[1]]);
     roll.val = ( roll_delta > 0) ? max(CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[2]],  roll.val - CONFIG.data.channelDeadzone[CONFIG.data.channelAssignment[2]]) : 
                                    min(CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[2]],  roll.val + CONFIG.data.channelDeadzone[CONFIG.data.channelAssignment[2]]);
      yaw.val = (  yaw_delta > 0) ? max(CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[3]],   yaw.val - CONFIG.data.channelDeadzone[CONFIG.data.channelAssignment[3]]) : 
                                    min(CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[3]],   yaw.val + CONFIG.data.channelDeadzone[CONFIG.data.channelAssignment[3]]);

    uint16_t throttle_threshold = ((throttle.max - throttle.min) / 10) + throttle.min; // keep bottom 10% of throttle stick to mean 'off'
 
    state->command_throttle = constrain((throttle.val - throttle_threshold) * 4095 / (throttle.max - throttle_threshold), 0, 4095);
    state->command_pitch =    constrain((1-2*((CONFIG.data.channelInversion >> 1) & 1))*(pitch.val - CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[1]]) * 4095 / (pitch.max - pitch.min), -2047, 2047);
    state->command_roll =     constrain((1-2*((CONFIG.data.channelInversion >> 2) & 1))*( roll.val - CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[2]]) * 4095 / ( roll.max -  roll.min), -2047, 2047);
    state->command_yaw =      constrain((1-2*((CONFIG.data.channelInversion >> 3) & 1))*(  yaw.val - CONFIG.data.channelMidpoint[CONFIG.data.channelAssignment[3]]) * 4095 / (  yaw.max -   yaw.min), -2047, 2047);
}
