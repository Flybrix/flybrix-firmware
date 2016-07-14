/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "command.h"

#include "config.h"  //CONFIG variable

#include "state.h"

#include "R415X.h"

#include "cardManagement.h"

PilotCommand::PilotCommand(State* __state, R415X* __receiver)
    : state(__state), receiver(__receiver) {
}

void PilotCommand::processCommands(void) {

    if (!(state->command_source_mask & COMMAND_READY_BTLE)){
        // if we aren't receiving bluetooth command data, try to get it from the R415X
        receiver->getCommandData(state);
    }

    if (!(state->command_source_mask & (COMMAND_READY_R415X | COMMAND_READY_BTLE))){
        // we have no command data!
        state->command_invalid_count += 2;
        if (state->command_invalid_count > 100) {
            state->command_invalid_count = 100;
            state->set(STATUS_RX_FAIL);
        } else if (state->command_invalid_count > 10) {
            state->set(STATUS_UNPAIRED);
        }
    } else if (state->command_invalid_count > 0) {
        state->command_invalid_count--;
        if (state->command_invalid_count == 0) {
            state->clear(STATUS_UNPAIRED);
        } else if (state->command_invalid_count < 90) {
            state->clear(STATUS_RX_FAIL);
        }
    }

    bool attempting_to_enable  = state->command_AUX_mask & (1 << 0);  // AUX1 is low
    bool attempting_to_disable = state->command_AUX_mask & (1 << 2);  // AUX1 is high

    if (blockEnabling && attempting_to_enable && !state->is(STATUS_OVERRIDE)) {  // user attempted to enable, but we are blocking it
        state->clear(STATUS_IDLE);
        state->set(STATUS_FAIL_STABILITY);
        state->set(STATUS_FAIL_ANGLE);  // set both flags as indication
    }
    blockEnabling = false;  // we block enable on the first run!
    if (!state->is(STATUS_OVERRIDE)) {
        if (attempting_to_enable && !state->is(STATUS_ENABLED | STATUS_FAIL_STABILITY | STATUS_FAIL_ANGLE)) {
            state->processMotorEnablingIteration();
            recentlyEnabled = true;
            throttleHoldOff = 80;  // @40Hz -- hold for 2 sec
            if (state->is(STATUS_ENABLED))
                sdcard::openFile();
        }
        if (attempting_to_disable && state->is(STATUS_ENABLED | STATUS_FAIL_STABILITY | STATUS_FAIL_ANGLE)) {
            state->disableMotors();
            sdcard::closeFile();
        }
    }

    bool throttle_is_low = (state->command_throttle == 0);

    if (recentlyEnabled || throttle_is_low) {
        state->command_throttle = 0;
        state->command_pitch    = 0;
        state->command_roll     = 0;
        state->command_yaw      = 0;

        throttleHoldOff--;
        if (recentlyEnabled && (throttleHoldOff == 0)) {
            recentlyEnabled = false;
        }
    }

    if (state->is(STATUS_OVERRIDE))
        blockEnabling = true;  // block enabling when we come out of pilot override

    // in the future, this would be the place to look for other combination inputs or for AUX levels that mean something
}
