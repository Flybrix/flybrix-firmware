/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "command.h"

#include "state.h"
#include "R415X.h"
#include "cardManagement.h"
#include "stateFlag.h"

PilotCommand::PilotCommand(State* __state, R415X* __receiver, StateFlag& flag) : state(__state), receiver(__receiver), flag_(flag) {
}

void PilotCommand::processCommands(void) {
    bool attempting_to_enable = false;
    bool attempting_to_disable = false;

    if (flag_.is(Status::RX_FAIL)) {
        state->command_throttle *= 0.99;
    }

    if (!(state->command_source_mask & COMMAND_READY_BTLE)) {
        if (bluetoothTolerance) {
            // we allow bluetooth a generous 1s before we give up
            --bluetoothTolerance;
        } else {
            // since we haven't seen bluetooth commands for more than 1 second, try the R415X
            receiver->getCommandData(state);
        }
    } else {
        // as soon as we start receiving bluetooth, reset the watchdog
        bluetoothTolerance = 40;
    }

    if (!(state->command_source_mask & (COMMAND_READY_R415X | COMMAND_READY_BTLE))) {
        // we have no command data!
        state->command_invalid_count++;
        if (state->command_invalid_count > 80) {
            // we haven't received data in two seconds
            state->command_invalid_count = 80;
            flag_.set(Status::RX_FAIL);
        }
    } else if (state->command_invalid_count > 0) {
        state->command_invalid_count--;
        if (state->command_invalid_count == 0) {
            flag_.clear(Status::RX_FAIL);
        }
    } else {
        // use valid command data
        attempting_to_enable = state->command_AUX_mask & (1 << 0);   // AUX1 is low
        attempting_to_disable = state->command_AUX_mask & (1 << 2);  // AUX1 is high

        // in the future, this would be the place to look for other combination inputs or for AUX levels that mean something

        // mark the BTLE data as used so we don't use it again
        state->command_source_mask &= ~(COMMAND_READY_BTLE);
    }

    if (blockEnabling && attempting_to_enable && !flag_.is(Status::OVERRIDE)) {  // user attempted to enable, but we are blocking it
        flag_.clear(Status::IDLE);
        flag_.set(Status::FAIL_OTHER);
    }
    blockEnabling = false;  // we only block enabling if attempting_to_enable may have been accidentally set

    if (!flag_.is(Status::OVERRIDE)) {
        if (attempting_to_enable && !flag_.is(Status::ENABLED | Status::FAIL_STABILITY | Status::FAIL_ANGLE | Status::FAIL_OTHER)) {
            state->processMotorEnablingIteration();  // this can flip Status::ENABLED to true
            recentlyEnabled = true;
            throttleHoldOff = 80;  // @40Hz -- hold for 2 sec
            if (flag_.is(Status::ENABLED))
                sdcard::openFile();
        }
        if (attempting_to_disable && flag_.is(Status::ENABLED | Status::FAIL_STABILITY | Status::FAIL_ANGLE | Status::FAIL_OTHER)) {
            state->disableMotors();
            sdcard::closeFile();
        }
    } else {
        blockEnabling = true;  // block accidental enabling when we come out of pilot override
    }

    bool throttle_is_low = (state->command_throttle == 0);

    if (recentlyEnabled || throttle_is_low) {
        state->command_throttle = 0;
        state->command_pitch = 0;
        state->command_roll = 0;
        state->command_yaw = 0;

        throttleHoldOff--;
        if (recentlyEnabled && (throttleHoldOff == 0)) {
            recentlyEnabled = false;
        }
    }
}
