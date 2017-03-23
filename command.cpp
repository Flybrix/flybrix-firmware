/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "command.h"

#include "airframe.h"
#include "systems.h"
#include "state.h"
#include "MPU9250.h"
#include "R415X.h"
#include "cardManagement.h"
#include "commandVector.h"
#include "stateFlag.h"

PilotCommand::PilotCommand(Systems& systems) : state_(systems.state), mpu_(systems.mpu), receiver_(systems.receiver), flag_(systems.flag), command_vector_(systems.command_vector) {
    setControlState(ControlState::AwaitingAuxDisable);
}

Airframe::MixTable& PilotCommand::mix_table() {
    return airframe_.mix_table;
}

void PilotCommand::override(bool override) {
    if (override) {
        setControlState(ControlState::Overridden);
    } else if (isOverridden()) {
        setControlState(ControlState::AwaitingAuxDisable);
    }
}

bool PilotCommand::isOverridden() const {
    return control_state_ == ControlState::Overridden;
}

void PilotCommand::setMotor(size_t index, uint16_t value) {
    if (isOverridden()) {
        airframe_.setMotor(index, value);
    }
}

void PilotCommand::resetMotors() {
    if (isOverridden()) {
        airframe_.resetMotors();
    }
}

void PilotCommand::applyControl(const ControlVectors& control_vectors) {
    airframe_.applyChanges(control_vectors);
}

void PilotCommand::processMotorEnablingIteration() {
    // Ignore if motors are already enabled
    if (!airframe_.motorsEnabled()) {
        processMotorEnablingIterationHelper();  // this can flip Status::ENABLED to true
        // hold controls low for some time after enabling
        throttle_hold_off_.reset(80);  // @40Hz -- hold for 2 sec
    }
}

void PilotCommand::processMotorEnablingIterationHelper() {
    if (!canRequestEnabling()) {
        return;
    }
    if (control_state_ != ControlState::Enabling) {
        setControlState(ControlState::Enabling);
        enable_attempts_ = 0;
    }

    if (enable_attempts_ == 0) {  // first call
        mpu_.forgetBiasValues();  // our filters will start filling with fresh values!
        enable_attempts_ = 1;
        return;
    }

    enable_attempts_++;  // we call this routine from "command" at 40Hz
    if (!state_.upright()) {
        setControlState(ControlState::FailAngle);
        return;
    }
    // wait ~1 seconds for the IIR filters to adjust to their bias free values
    if (enable_attempts_ == 41) {
        if (!state_.stable()) {
            setControlState(ControlState::FailStability);
        } else {
            mpu_.correctBiasValues();  // now our filters will start filling with accurate data
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
            setControlState(ControlState::FailStability);
        } else {
            setControlState(ControlState::ThrottleLocked);
            sdcard::openFile();
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
    setControlState(ControlState::Disabled);
    sdcard::closeFile();
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
        if (!isOverridden()) {
            setControlState(ControlState::FailRx);
        }
    } else if (invalid_count < 0) {
        invalid_count = 0;
        if (control_state_ == ControlState::FailRx) {
            setControlState(ControlState::AwaitingAuxDisable);
        }
    }

    bool attempting_to_enable{(command_vector_.aux_mask & (1 << 0)) != 0};   // AUX1 is low
    bool attempting_to_disable{(command_vector_.aux_mask & (1 << 2)) != 0};  // AUX1 is high

    switch (control_state_) {
        case ControlState::Overridden: {
        } break;
        case ControlState::Disabled: {
            if (attempting_to_enable) {
                setControlState(ControlState::Enabling);
                enable_attempts_ = 0;
            }
        } break;
        case ControlState::ThrottleLocked: {
            if (!attempting_to_enable) {
                setControlState(ControlState::Disabled);
            } else if (command_vector_.throttle == 0) {
                setControlState(ControlState::Enabled);
            }
        } break;
        case ControlState::FailStability:
        case ControlState::FailAngle:
        case ControlState::AwaitingAuxDisable:
        case ControlState::Enabling:
        case ControlState::Enabled: {
            if (!attempting_to_enable) {
                setControlState(ControlState::Disabled);
            }
        } break;
        case ControlState::FailRx: {
            command_vector_.throttle *= 0.99;
        } break;
    }

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

void PilotCommand::setControlState(ControlState state) {
    control_state_ = state;

    airframe_.setOverride(isOverridden());

    flag_.assign(Status::IDLE, control_state_ == ControlState::Disabled);
    flag_.assign(Status::ENABLING, control_state_ == ControlState::Enabling);
    flag_.assign(Status::FAIL_OTHER, control_state_ == ControlState::AwaitingAuxDisable || control_state_ == ControlState::ThrottleLocked);
    flag_.assign(Status::FAIL_STABILITY, control_state_ == ControlState::FailStability);
    flag_.assign(Status::FAIL_ANGLE, control_state_ == ControlState::FailAngle);
    flag_.assign(Status::RX_FAIL, control_state_ == ControlState::FailRx);
    flag_.assign(Status::ENABLED, control_state_ == ControlState::Enabled);
    flag_.assign(Status::OVERRIDE, isOverridden());
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
