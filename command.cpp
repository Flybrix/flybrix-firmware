/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "command.h"

#include "BMP280.h"
#include "cardManagement.h"
#include "debug.h"
#include "imu.h"
#include "led.h"
#include "state.h"
#include "stateFlag.h"
#include "systems.h"

namespace {
// RX-loss failsafe, evaluated at the 40Hz pilot input rate. These are starting values: check them with a tethered test.
// Each tick the throttle is multiplied by this (0.99 -> ~-33% per second), so the craft settles into a descent.
constexpr float FAILSAFE_THROTTLE_DECAY = 0.99f;
// Below this the controller treats the throttle as off anyway.
constexpr float FAILSAFE_CUTOFF_THROTTLE = 10.0f;
// Cut the motors after this long in failsafe even if the throttle has not decayed yet (400 ticks = 10s).
constexpr uint16_t FAILSAFE_MAX_TICKS = 400;
// Holding level, a larger tilt than this means the craft has crashed or tumbled: cut the motors.
constexpr float FAILSAFE_CRASH_ANGLE_RAD = 60.0f * 0.01745329252f;
}  // namespace

PilotCommand::PilotCommand(Systems& systems) : bmp_(systems.bmp), state_(systems.state), imu_(systems.imu), flag_(systems.flag), led_{systems.led} {
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
        processMotorEnablingIterationHelper();  // this can flip Status::ARMED to true
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

    // never arm on frozen sensor data: the upright/stable checks below would pass on stale values
    if (!imu_.healthy(ClockTime::now())) {
        setControlState(ControlState::FailStability);
        return;
    }

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
            sdcard::writing::open();
            airframe_.enableMotors();
            flag_.assign(Status::RECORDING_SD, sdcard::getState() == sdcard::State::WriteStates);
            setControlState(ControlState::ThrottleLocked);
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

void PilotCommand::cutMotors() {
    airframe_.disableMotors();
    sdcard::writing::close();
    flag_.assign(Status::RECORDING_SD, sdcard::getState() == sdcard::State::WriteStates);
}

void PilotCommand::disableMotors() {
    cutMotors();
    setControlState(ControlState::Disabled);
}

void PilotCommand::emergencyDisarm() {
    if (isOverridden() || !airframe_.motorsEnabled()) {
        return;
    }
    cutMotors();
    // an arming-failure state: the pilot has to cycle the arm switch before the motors can be enabled again
    setControlState(ControlState::FailStability);
}

void PilotCommand::startFailsafe() {
    // Only descend if we were actually flying; otherwise there is nothing to keep spinning.
    bool flying{airframe_.motorsEnabled() && control_state_ == ControlState::Enabled};
    failsafe_throttle_ = flying ? last_command_.throttle : 0.0f;
    failsafe_ticks_ = 0;
    setControlState(ControlState::FailRx);
}

void PilotCommand::endFailsafe(const RcCommand& command) {
    if (airframe_.motorsEnabled() && command.aux1 == RcCommand::AUX::Low) {
        // Still in the air and the pilot is still asking to be armed: hand control back.
        // (This used to go to AwaitingAuxDisable with the motors still live, while the LEDs showed "not armed".)
        setControlState(ControlState::Enabled);
    } else {
        cutMotors();
        setControlState(ControlState::AwaitingAuxDisable);
    }
}

RcCommand PilotCommand::failsafeCommand() {
    ++failsafe_ticks_;
    failsafe_throttle_ *= FAILSAFE_THROTTLE_DECAY;

    if (airframe_.motorsEnabled()) {
        const Kinematics& kinematics = state_.getKinematics();
        bool crashed{fabsf(kinematics.angle.pitch) > FAILSAFE_CRASH_ANGLE_RAD || fabsf(kinematics.angle.roll) > FAILSAFE_CRASH_ANGLE_RAD};
        if (failsafe_throttle_ < FAILSAFE_CUTOFF_THROTTLE || failsafe_ticks_ > FAILSAFE_MAX_TICKS || crashed) {
            cutMotors();
        }
    }
    if (!airframe_.motorsEnabled()) {
        failsafe_throttle_ = 0.0f;
    }

    // Level attitude, no yaw, decaying throttle. (Previously the timed-out source delivered an all-zero command,
    // so the throttle decay had nothing to act on and the motors stopped instantly.)
    RcCommand command;
    command.aux1 = last_command_.aux1;
    command.aux2 = last_command_.aux2;
    command.throttle = (int16_t)failsafe_throttle_;
    return command;
}

uint8_t count{0};

RcCommand PilotCommand::processCommands(RcState&& rc_state) {
    bool timeout{rc_state.status == RcStatus::Timeout};

    if (timeout) {
        if (!isOverridden() && control_state_ != ControlState::FailRx) {
            startFailsafe();
        }
    } else if (control_state_ == ControlState::FailRx) {
        endFailsafe(rc_state.command);
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
                count++;  // one check isn't enough when opening an sd card (zero received before loop delay?)
                if (count > 10) {
                    setControlState(ControlState::Enabled);
                    count = 0;
                }
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
            rc_state.command = failsafeCommand();
        } break;
    }

    if (control_state_ == ControlState::Enabling) {
        processMotorEnablingIteration();
    } else if (control_state_ == ControlState::Disabled) {
        disableMotors();
    }

    bool zeroed{false};
    if (!timeout) {
        if (throttle_hold_off_.tick() || rc_state.command.throttle == 0 || control_state_ == ControlState::ThrottleLocked) {
            rc_state.command.throttle = 0;
            rc_state.command.pitch = 0;
            rc_state.command.roll = 0;
            rc_state.command.yaw = 0;
            zeroed = true;
        }
        last_command_ = rc_state.command;
    }
    sticks_pass_through_ = !timeout && !zeroed && control_state_ == ControlState::Enabled;

    return rc_state.command;
}

RcCommand PilotCommand::refreshSticks(const RcCommand& fresh, const RcCommand& current) const {
    if (!sticks_pass_through_) {
        return current;
    }
    RcCommand command = current;  // keeps the aux (arm/mode) switches from the state machine
    command.throttle = fresh.throttle;
    command.pitch = fresh.pitch;
    command.roll = fresh.roll;
    command.yaw = fresh.yaw;
    return command;
}

bool PilotCommand::isArmingFailureState() const {
    return control_state_ == ControlState::FailAngle || control_state_ == ControlState::FailStability || control_state_ == ControlState::AwaitingAuxDisable ||
           control_state_ == ControlState::ThrottleLocked;
}

uint32_t PilotCommand::failToColor() const {
    switch (control_state_) {
        case ControlState::FailStability:
            return CRGB::Red;
        case ControlState::FailAngle:
            return CRGB::Orange;
        case ControlState::AwaitingAuxDisable:
        case ControlState::ThrottleLocked:
            return CRGB::White;
        default:
            return CRGB::Black;
    }
}

void PilotCommand::setControlState(ControlState state) {
    control_state_ = state;
    if (isArmingFailureState()) {
        CRGB color = CRGB::White;
        for (const LED::StateCase& sc : led_.states.states) {
            if (sc.status & Status::ARMING) {
                color = sc.color_right_front.crgb();
                break;
            }
        }
        led_.errorStart(LEDPattern::ALTERNATE, color, LED::fade(failToColor()), 2);
    } else {
        led_.errorStop();
    }

    airframe_.setOverride(isOverridden());

    flag_.assign(Status::IDLE, control_state_ == ControlState::Disabled);
    flag_.assign(Status::ARMING, control_state_ == ControlState::Enabling || isArmingFailureState());
    flag_.assign(Status::NO_SIGNAL, control_state_ == ControlState::FailRx);
    flag_.assign(Status::ARMED, control_state_ == ControlState::Enabled);
    flag_.assign(Status::OVERRIDE, isOverridden());
}
