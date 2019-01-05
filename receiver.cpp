/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "receiver.h"
#include "board.h"
#include "debug.h"

// RX -- PKZ3341 sends: RHS left/right, RHS up/down, LHS up/down, LHS
// left/right, RHS click (latch), LHS button(momentary)
// map throttle to LHS up/down
// map pitch to RHS up/down
// map roll to RHS left/righ
// map yaw to LHS up/down
// map AUX1 to RHS click
// map AUX2 to LHS click

Receiver::ChannelProperties::ChannelProperties() : assignment{2, 1, 0, 3, 4, 5}, inversion{6}, midpoint{1500, 1500, 1500, 1500, 1500, 1500}, deadzone{0, 0, 0, 20, 0, 0} {
}

volatile uint8_t  RX_freshness = 0;
volatile uint16_t RX[RC_CHANNEL_COUNT];         // filled by the interrupt with valid data
volatile uint16_t RX_errors = 0;                // count dropped frames
volatile uint16_t startPulse = 0;               // keeps track of the last received pulse position
volatile uint16_t RX_buffer[RC_CHANNEL_COUNT];  // buffer data in anticipation of a valid frame
volatile uint8_t  RX_channel = 0;               // we are collecting data for this channel

bool Receiver::ChannelProperties::verify() const {
    bool ok{true};
    bool assigned[] = {false, false, false, false, false, false};
    for (size_t idx = 0; idx < 6; ++idx) {
        uint8_t assig = assignment[idx];
        if (assig < 0 || assig > 5) {
            ok = false;
            DebugPrint("All channel assignments must be within the [0, 5] range");
        } else if (assigned[assig]) {
            ok = false;
            DebugPrint("Duplicate channel assignment detected");
        } else {
            assigned[assig] = true;
        }
    }
    for (size_t idx = 0; idx < 6; ++idx) {
        if (midpoint[idx] < PPMchannel::min || midpoint[idx] > PPMchannel::max) {
            ok = false;
            DebugPrint("Channel midpoints must be within the channel range");
        }
        if (midpoint[idx] < deadzone[idx]) {
            ok = false;
            DebugPrint("Channel deadzone cannot be larger than the midpoint value");
        }
    }
    return ok;
}

Receiver::Receiver() {
    pinMode(board::RX_DAT, INPUT);  // WE ARE ASSUMING RX_DAT IS PIN 3 IN FTM1 SETUP!

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
                RX_freshness = 20;
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

void Receiver::updateChannels() {
    updateChannel(0, throttle);
    updateChannel(1, pitch);
    updateChannel(2, roll);
    updateChannel(3, yaw);
    updateChannel(4, AUX1);
    updateChannel(5, AUX2);
}

void Receiver::updateChannel(std::size_t idx, PPMchannel& target) {
    uint8_t assignment = channel.assignment[idx];
    // read data into PPMchannel objects using receiver channels assigned from configuration
    target.val = RX[assignment];
    // update midpoints from config
    target.mid = channel.midpoint[assignment];
    // update deadzones from config
    target.deadzone = channel.deadzone[assignment];
    // update channel inversion from config
    target.inverted = channel.inversion & (1 << idx);
}

RcState Receiver::query() {
    RcState rc_state;
    cli();  // disable interrupts

    bool input_is_ready = (RX_freshness > 0);
    
    if (input_is_ready) {
        --RX_freshness;

        // if receiver is working, we should never see anything less than 900!
        for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
            if (RX[i] < 900) {
                input_is_ready = false;
                break;
            }
        }
    }

    if (!input_is_ready) {
        // tell state that Receiver is not ready and return
        sei();  // enable interrupts
        rc_state.status = RcStatus::Timeout;
        return rc_state;
    }

    updateChannels();

    sei();  // enable interrupts

    // Translate PPMChannel data into the four command level and aux mask
    rc_state.status = RcStatus::Ok;

    rc_state.command.parseBools(AUX1.isLow(), AUX1.isMid(), AUX1.isHigh(), AUX2.isLow(), AUX2.isMid(), AUX2.isHigh());

    // in some cases it is impossible to get a ppm channel to be close enought to the midpoint (~1500 usec) because the controller trim is too coarse to correct a small error
    // we get around this by creating a small dead zone around the midpoint of signed channel, specified in usec units
    pitch.trimDeadzone();
    roll.trimDeadzone();
    yaw.trimDeadzone();

    rc_state.command.throttle = throttle.decodeAbsoluteWithThreshold();
    rc_state.command.pitch = pitch.decodeOffset();
    rc_state.command.roll = roll.decodeOffset();
    rc_state.command.yaw = yaw.decodeOffset();

    return rc_state;
}
