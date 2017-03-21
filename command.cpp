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
    // Ignore if motors are already enabled
    if (!airframe_.motorsEnabled()) {
        processMotorEnablingIterationHelper();  // this can flip Status::ENABLED to true
        // hold controls low for some time after enabling
        throttle_hold_off_.reset(80);  // @40Hz -- hold for 2 sec
        if (airframe_.motorsEnabled()) {
            control_state_ = ControlState::ThrottleLocked;
            sdcard::openFile();
        }
    }
    updateControlStateFlags();
}

void PilotCommand::processMotorEnablingIterationHelper() {
    if (!canRequestEnabling()) {
        return;
    }
    if (control_state_ != ControlState::Enabling) {
        control_state_ = ControlState::Enabling;
        enable_attempts_ = 0;
    }

    if (enable_attempts_ == 0) {            // first call
        flag_.set(Status::CLEAR_MPU_BIAS);  // our filters will start filling with fresh values!
        enable_attempts_ = 1;
        return;
    }

    enable_attempts_++;  // we call this routine from "command" at 40Hz
    if (!state_.upright()) {
        control_state_ = ControlState::FailAngle;
        return;
    }
    // wait ~1 seconds for the IIR filters to adjust to their bias free values
    if (enable_attempts_ == 41) {
        if (!state_.stable()) {
            control_state_ = ControlState::FailStability;
        } else {
            flag_.set(Status::SET_MPU_BIAS);  // now our filters will start filling with accurate
        }
        return;
    }

    // reset the filter to start letting state reconverge with bias corrected mpu data
    if (enable_attempts_ == 42) {
        state_.resetState();
        return;
    }
    // wait ~1 seconds for the state filter to converge
    // check one more time to see if we were stable
    if (enable_attempts_ > 81) {
        if (!state_.stable()) {
            control_state_ = ControlState::FailStability;
        } else {
            airframe_.enableMotors();
        }
        return;
    }
}

bool PilotCommand::canRequestEnabling() const {
    switch (control_state_) {
        case ControlState::ThrottleLocked:
        case ControlState::Enabled:
        case ControlState::FailStability:
        case ControlState::FailAngle:
        case ControlState::FailRx:
        case ControlState::Overridden:
            return false;
        case ControlState::AwaitingAuxDisable:
        case ControlState::Disabled:
        case ControlState::Enabling:
            return true;
    }
    return true;
}

void PilotCommand::disableMotors() {
    airframe_.disableMotors();
    control_state_ = ControlState::Disabled;
    sdcard::closeFile();
    updateControlStateFlags();
}

void PilotCommand::processCommands() {
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
        control_state_ = ControlState::FailRx;
    } else if (invalid_count < 0) {
        invalid_count = 0;
        if (control_state_ == ControlState::FailRx) {
            control_state_ = ControlState::AwaitingAuxDisable;
        }
    }

    bool attempting_to_enable{(command_vector_.aux_mask & (1 << 0)) != 0};   // AUX1 is low
    bool attempting_to_disable{(command_vector_.aux_mask & (1 << 2)) != 0};  // AUX1 is high
    bool override{airframe_.motorsOverridden()};

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
                enable_attempts_ = 0;
            }
        } break;
        case ControlState::ThrottleLocked: {
            if (!attempting_to_enable) {
                control_state_ = ControlState::Disabled;
            } else if (command_vector_.throttle == 0) {
                control_state_ = ControlState::Enabled;
            }
        } break;
        case ControlState::FailStability:
        case ControlState::FailAngle:
        case ControlState::AwaitingAuxDisable:
        case ControlState::Enabling:
        case ControlState::Enabled: {
            if (!attempting_to_enable) {
                control_state_ = ControlState::Disabled;
            }
        } break;
        case ControlState::FailRx: {
            command_vector_.throttle *= 0.99;
        } break;
    }

    updateControlStateFlags();

    if (control_state_ == ControlState::Enabling) {
        processMotorEnablingIteration();
    } else if (control_state_ == ControlState::Disabled) {
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
    flag_.clear(Status::IDLE | Status::ENABLING | Status::FAIL_OTHER | Status::FAIL_STABILITY | Status::FAIL_ANGLE | Status::RX_FAIL);
    switch (control_state_) {
        case ControlState::AwaitingAuxDisable:
        case ControlState::ThrottleLocked:
            flag_.set(Status::FAIL_OTHER);
            break;
        case ControlState::Enabling:
            flag_.set(Status::ENABLING);
            break;
        case ControlState::Overridden:
        case ControlState::Disabled:
            flag_.set(Status::IDLE);
            break;
        case ControlState::FailStability:
            flag_.set(Status::FAIL_STABILITY);
            break;
        case ControlState::FailAngle:
            flag_.set(Status::FAIL_ANGLE);
            break;
        case ControlState::FailRx:
            flag_.set(Status::RX_FAIL);
            break;
        case ControlState::Enabled:
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
