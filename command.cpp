/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "command.h"

#include "BMP280.h"
#include "systems.h"
#include "state.h"
#include "imu.h"
#include "cardManagement.h"
#include "stateFlag.h"
#include "debug.h"

PilotCommand::PilotCommand(Systems& systems) : bmp_(systems.bmp), state_(systems.state), imu_(systems.imu), flag_(systems.flag) {
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

bool PilotCommand::upright() const {
    return imu_.upright();
}

bool PilotCommand::stable() const {
    return imu_.stable();
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
        enable_attempts_ = 1;
        return;
    }

    enable_attempts_++;  // we call this routine from "command" at 40Hz
    
    if (!upright()) {
        setControlState(ControlState::FailAngle);
        return;
    }
    
    if (enable_attempts_ == 21) {
        if (!stable()) {
            setControlState(ControlState::FailStability);
        } else {
            imu_.readBiasValues();  // update filter values if they are not fresh
            bmp_.recalibrateP0();
        }
        return;
    }
    
    // check one more time to see if we were stable
    if (enable_attempts_ == 41) {
        if (!stable()) {
            setControlState(ControlState::FailStability);
        } else {
            setControlState(ControlState::ThrottleLocked);
            sdcard::writing::open();
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
    sdcard::writing::close();
}

RcCommand PilotCommand::processCommands(RcState&& rc_state) {
    bool timeout{rc_state.status == RcStatus::Timeout};

    if (timeout) {
        if (!isOverridden()) {
            setControlState(ControlState::FailRx);
        }
    } else {
        if (control_state_ == ControlState::FailRx) {
            setControlState(ControlState::AwaitingAuxDisable);
        }
    }

    bool attempting_to_enable{rc_state.command.aux1 == RcCommand::AUX::Low};

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
            } else if (rc_state.command.throttle == 0) {
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
            rc_state.command.throttle *= 0.99;
        } break;
    }

    if (control_state_ == ControlState::Enabling) {
        processMotorEnablingIteration();
    } else if (control_state_ == ControlState::Disabled) {
        disableMotors();
    }

    if (!timeout) {
        if (throttle_hold_off_.tick() || rc_state.command.throttle == 0 || control_state_ == ControlState::ThrottleLocked) {
            rc_state.command.throttle = 0;
            rc_state.command.pitch = 0;
            rc_state.command.roll = 0;
            rc_state.command.yaw = 0;
        }
    }

    return rc_state.command;
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
