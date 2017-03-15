/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "command.h"

#include "airframe.h"
#include "systems.h"
#include "state.h"
#include "R415X.h"
#include "cardManagement.h"
#include "commandVector.h"
#include "stateFlag.h"

PilotCommand::PilotCommand(Systems& systems) : airframe_(systems.airframe), state_(systems.state), receiver_(systems.receiver), flag_(systems.flag), command_vector_(systems.command_vector) {
}

void PilotCommand::processMotorEnablingIteration() {
    processMotorEnablingIterationHelper();  // this can flip Status::ENABLED to true
    // hold controls low for some time after enabling
    throttle_hold_off_.reset(80);  // @40Hz -- hold for 2 sec
    control_state_ = ControlState::Enabling;
    if (flag_.is(Status::ENABLED)) {
        control_state_ = ControlState::ThrottleLocked;
        sdcard::openFile();
    }
    updateControlStateFlags();
}

void PilotCommand::processMotorEnablingIterationHelper() {
    if (airframe_.motorsEnabled()) {  // lazy GUI calls...
        // ERROR: ("DEBUG: extra call to processMotorEnablingIteration()!");
    } else if (flag_.is(Status::IDLE)) {  // first call
        flag_.clear(Status::IDLE);
        flag_.set(Status::ENABLING);
        flag_.set(Status::CLEAR_MPU_BIAS);  // our filters will start filling with fresh values!
        enable_attempts_ = 0;
    } else if (flag_.is(Status::ENABLING)) {
        enable_attempts_++;  // we call this routine from "command" at 40Hz
        if (!state_.upright()) {
            flag_.clear(Status::ENABLING);
            flag_.set(Status::FAIL_ANGLE);
        }
        // wait ~1 seconds for the IIR filters to adjust to their bias free values
        if (enable_attempts_ == 40) {
            if (!state_.stable()) {
                flag_.clear(Status::ENABLING);
                flag_.set(Status::FAIL_STABILITY);
            } else {
                flag_.set(Status::SET_MPU_BIAS);  // now our filters will start filling with accurate
            }

        } else if (enable_attempts_ == 41) {  // reset the filter to start letting state reconverge with bias corrected mpu data
            state_.resetState();
        }
        // wait ~1 seconds for the state filter to converge
        else if (enable_attempts_ > 80) {  // check one more time to see if we were stable
            if (!state_.stable()) {
                flag_.clear(Status::ENABLING);
                flag_.set(Status::FAIL_STABILITY);

            } else {
                flag_.clear(Status::ENABLING);
                airframe_.enableMotors();
            }
        }
    }
}

void PilotCommand::disableMotors() {
    airframe_.disableMotors();
    control_state_ = ControlState::Disabled;
    sdcard::closeFile();
    updateControlStateFlags();
}

void PilotCommand::processCommands() {
    bool attempting_to_enable = false;
    bool attempting_to_disable = false;

    if (flag_.is(Status::RX_FAIL)) {
        command_vector_.throttle *= 0.99;
    }

    if (command_vector_.source != CommandVector::Source::Bluetooth) {
        if (!bluetooth_tolerance_.tick()) {
            // since we haven't seen bluetooth commands for more than 1 second, try the R415X
            command_vector_ = receiver_.getCommandData();
        }
    } else {
        // we allow bluetooth a generous 1s before we give up
        // as soon as we start receiving bluetooth, reset the watchdog
        bluetooth_tolerance_.reset(40);
    }

    bool has_data{command_vector_.source != CommandVector::Source::None};

    switch (command_vector_.source) {
        case CommandVector::Source::Radio:
            --invalid_count;
            break;
        case CommandVector::Source::Bluetooth:
            // We tolerate bluetooth working as bad as 1 in 10 times
            invalid_count -= 10;
            break;
        default:
            ++invalid_count;
    }

    // mark the command data as used so we don't use it again
    command_vector_.source = CommandVector::Source::None;

    if (invalid_count > 80) {
        invalid_count = 80;
        flag_.set(Status::RX_FAIL);
    } else if (invalid_count < 0) {
        invalid_count = 0;
        flag_.clear(Status::RX_FAIL);
    }

    if (!flag_.is(Status::RX_FAIL)) {
        attempting_to_enable = command_vector_.aux_mask & (1 << 0);   // AUX1 is low
        attempting_to_disable = command_vector_.aux_mask & (1 << 2);  // AUX1 is high
    }

    bool override{flag_.is(Status::OVERRIDE)};

    if (override) {
        control_state_ = ControlState::Overridden;
    }

    switch (control_state_) {
        case ControlState::Overridden: {
            if (!override) {
                control_state_ = ControlState::AwaitingAuxDisable;
            }
        } break;
        case ControlState::Disabled: {
            if (attempting_to_enable) {
                control_state_ = ControlState::Enabling;
            }
        } break;
        case ControlState::ThrottleLocked: {
            if (!attempting_to_enable) {
                control_state_ = ControlState::Disabled;
            } else if (command_vector_.throttle == 0) {
                control_state_ = ControlState::Enabled;
            }
        } break;
        case ControlState::AwaitingAuxDisable:
        case ControlState::Enabling:
        case ControlState::Enabled: {
            if (!attempting_to_enable) {
                control_state_ = ControlState::Disabled;
            }
        } break;
    }

    updateControlStateFlags();

    if (control_state_ == ControlState::Enabling && !flag_.is(Status::ENABLED | Status::FAIL_STABILITY | Status::FAIL_ANGLE)) {
        processMotorEnablingIteration();
    } else if (control_state_ == ControlState::Disabled && flag_.is(Status::ENABLED | Status::FAIL_STABILITY | Status::FAIL_ANGLE)) {
        disableMotors();
    }

    if (has_data) {
        if (throttle_hold_off_.tick() || command_vector_.throttle == 0 || control_state_ == ControlState::ThrottleLocked) {
            command_vector_.throttle = 0;
            command_vector_.pitch = 0;
            command_vector_.roll = 0;
            command_vector_.yaw = 0;
        }
    }
}

void PilotCommand::updateControlStateFlags() {
    switch (control_state_) {
        case ControlState::AwaitingAuxDisable:
        case ControlState::ThrottleLocked:
            flag_.clear(Status::IDLE);
            flag_.set(Status::FAIL_OTHER);
            break;
        case ControlState::Enabled:
        case ControlState::Enabling:
            flag_.clear(Status::FAIL_OTHER);
            break;
        case ControlState::Overridden:
            flag_.set(Status::IDLE);
            flag_.clear(Status::FAIL_OTHER);
            break;
        case ControlState::Disabled:
            flag_.set(Status::IDLE);
            flag_.clear(Status::FAIL_STABILITY | Status::FAIL_ANGLE | Status::FAIL_OTHER);
            break;
    }
}

bool PilotCommand::Ticker::tick() {
    if (count_ == 0) {
        return false;
    }
    --count_;
    return true;
}

void PilotCommand::Ticker::reset(uint8_t ticks) {
    count_ = ticks;
}
