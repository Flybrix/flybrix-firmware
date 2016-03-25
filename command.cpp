/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "command.h"

#include "config.h"  //CONFIG variable

#include "state.h"

PilotCommand::PilotCommand(State* __state)
    : state{__state}, throttle_command{&state->command_throttle}, pitch_command{&state->command_pitch}, roll_command{&state->command_roll}, yaw_command{&state->command_yaw} {
}

void PilotCommand::loadRxData() {
    // read data into variables using channels assigned from configurator
    cli();  // disable interrupts

    // if R415X is working, we should never see anything less than 900!
    for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
        if (RX[i] < 900) {
            state->invalidRXcount += 2;
            if (state->invalidRXcount > 100) {
                state->invalidRXcount = 100;
                state->set(STATUS_RX_FAIL);
            } else if (state->invalidRXcount > 10) {
                state->set(STATUS_UNPAIRED);
            }
        } else if (state->invalidRXcount > 0) {
            state->invalidRXcount--;
            if (state->invalidRXcount == 0) {
                state->clear(STATUS_UNPAIRED);
            } else if (state->invalidRXcount < 90) {
                state->clear(STATUS_RX_FAIL);
            }
        }
    }

    if (state->invalidRXcount == 0) {
        // do not use bad data!
        throttle.update(RX[CONFIG.data.assignedChannel[0]]);
        pitch.update(RX[CONFIG.data.assignedChannel[1]]);
        roll.update(RX[CONFIG.data.assignedChannel[2]]);
        yaw.update(RX[CONFIG.data.assignedChannel[3]]);
        AUX1.update(RX[CONFIG.data.assignedChannel[4]]);
        AUX2.update(RX[CONFIG.data.assignedChannel[5]]);
    }

    sei();  // enable interrupts

    // Set the AUX bit mask
    // bitfield order is {AUX1_low, AUX1_mid, AUX1_high, AUX2_low, AUX2_mid, AUX2_high, x, x} (LSB-->MSB)
    
    state->AUX_chan_mask = 0x00;  // reset the AUX mode bitmask

    if (AUX1.isLow()) {
        state->AUX_chan_mask |= (1 << 0);
    } else if (AUX1.isMid()) {
        state->AUX_chan_mask |= (1 << 1);
    } else if (AUX1.isHigh()) {
        state->AUX_chan_mask |= (1 << 2);
    }
    if (AUX2.isLow()) {
        state->AUX_chan_mask |= (1 << 3);
    } else if (AUX2.isMid()) {
        state->AUX_chan_mask |= (1 << 4);
    } else if (AUX2.isHigh()) {
        state->AUX_chan_mask |= (1 << 5);
    }
}

void PilotCommand::processCommands(void) {
    loadRxData();  // DO THIS FIRST!

    if (blockEnabling && AUX1.isLow() && !state->is(STATUS_OVERRIDE)) {  // user attempted to enable, but we are blocking it
        state->clear(STATUS_IDLE);
        state->set(STATUS_FAIL_STABILITY);
        state->set(STATUS_FAIL_ANGLE);  // set both flags as indication
    }
    blockEnabling = false;  // we block enable on the first run!

    if (AUX1.isLow() && !state->is(STATUS_OVERRIDE)) {
        if (!state->is(STATUS_ENABLED) && !state->is(STATUS_FAIL_STABILITY) && !state->is(STATUS_FAIL_ANGLE)) {
            state->processMotorEnablingIteration();
            recentlyEnabled = true;
            throttleHoldOff = 80;  // @40Hz -- hold for 2 sec
        }
    } else if (AUX1.isHigh() && !state->is(STATUS_OVERRIDE)) {
        if (state->is(STATUS_ENABLED) || state->is(STATUS_FAIL_STABILITY) || state->is(STATUS_FAIL_ANGLE)) {
            state->disableMotors();
        }
    }

    if (recentlyEnabled || throttle.isLow()) {
        *throttle_command = 0;
        *pitch_command = 0;
        *roll_command = 0;
        *yaw_command = 0;

        throttleHoldOff--;
        if (recentlyEnabled && (throttleHoldOff == 0)) {
            recentlyEnabled = false;
        }
    } else {
        //
        // ppm channels range from about 900-1200 usec, but we want to keep the bottom ~10% of throttle reserved for "off"
        // users can modify the controller's output using the trim settings in a way that defeats this throttle "off" range!
        // this gives us a range of ~1080... we're scaling this received ppm range to 4095 because that's our full scale motor resolution
        // commandInversion is separate from the scaling factor in the controller because we would apply a negative scale factor twice in a cascade

        uint16_t throttle_threshold = ((throttle.max - throttle.min) / 10) + throttle.min;
        *throttle_command = constrain((throttle.val - throttle_threshold) * 4095 / (throttle.max - throttle_threshold), 0, 4095);
        *pitch_command =    constrain((1-2*((CONFIG.data.commandInversion >> 0) & 1))*(pitch.val - CONFIG.data.channelMidpoint[CONFIG.data.assignedChannel[1]]) * 4095 / (pitch.max - pitch.min), -2047, 2047);
        *roll_command =     constrain((1-2*((CONFIG.data.commandInversion >> 1) & 1))*( roll.val - CONFIG.data.channelMidpoint[CONFIG.data.assignedChannel[2]]) * 4095 / (roll.max - roll.min), -2047, 2047);
        *yaw_command =      constrain((1-2*((CONFIG.data.commandInversion >> 2) & 1))*(  yaw.val - CONFIG.data.channelMidpoint[CONFIG.data.assignedChannel[3]]) * 4095 / (yaw.max - yaw.min), -2047, 2047);
        
        //
        // in some cases it is impossible to get a ppm channel to be exactly 1500 usec because the controller trim is too coarse to correct a small error
        // we can get around by creating a small dead zone on the commands that are potentially effected
        
        *pitch_command = *pitch_command > 0 ? max(0, *pitch_command - CONFIG.data.channelDeadzone[CONFIG.data.assignedChannel[1]]) : min(*pitch_command + CONFIG.data.channelDeadzone[CONFIG.data.assignedChannel[1]], 0);
        *roll_command  = *roll_command > 0  ? max(0, *roll_command  - CONFIG.data.channelDeadzone[CONFIG.data.assignedChannel[2]]) : min(*roll_command  + CONFIG.data.channelDeadzone[CONFIG.data.assignedChannel[2]], 0);
        *yaw_command   = *yaw_command > 0   ? max(0, *yaw_command   - CONFIG.data.channelDeadzone[CONFIG.data.assignedChannel[3]]) : min(*yaw_command   + CONFIG.data.channelDeadzone[CONFIG.data.assignedChannel[3]], 0);

    }

    if (state->is(STATUS_OVERRIDE))
        blockEnabling = true;  // block enabling when we come out of pilot override

    // in the future, this would be the place to look for other combination inputs or for AUX levels that mean something
}
