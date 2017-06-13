/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <command.h/cpp>

    This code interprets the raw RC data into pilot intentions (stick inputs, mode changes, etc.)

*/

#ifndef command_h
#define command_h

#include <Arduino.h>

#include "airframe.h"
#include "R415X.h"

#include "utility/ticker.h"

struct Systems;
struct CommandVector;
struct ControlVectors;
class State;
class StateFlag;
class Imu;

class PilotCommand {
   public:
    explicit PilotCommand(Systems& systems);
    void processCommands();
    void processMotorEnablingIteration();
    void disableMotors();
    void override(bool override);
    bool isOverridden() const;
    void applyControl(const ControlVectors& control_vectors);
    void setMotor(size_t index, uint16_t value);
    void resetMotors();

    template <typename Tstream>
    bool readMotor(size_t index, Tstream& input) {
        return airframe_.readMotor(index, input);
    }

    template <typename Tstream>
    void writeMotorsTo(Tstream& output) const {
        return airframe_.writeMotorsTo(output);
    }

    Airframe::MixTable& mix_table();
    R415X& receiver();

   private:
    enum class ControlState {
        Overridden,
        AwaitingAuxDisable,
        Disabled,
        Enabling,
        ThrottleLocked,
        Enabled,
        FailStability,
        FailAngle,
        FailRx,
    };

    void processMotorEnablingIterationHelper();
    bool canRequestEnabling() const;
    void setControlState(ControlState state);

    State& state_;
    Imu& imu_;
    R415X receiver_;
    StateFlag& flag_;
    CommandVector& command_vector_;
    CommandSources& command_sources_;

    Airframe airframe_;
    ControlState control_state_{ControlState::AwaitingAuxDisable};
    Ticker<uint8_t> throttle_hold_off_;  // hold controls low for some time after enabling
    Ticker<uint8_t> bluetooth_tolerance_;
    int16_t invalid_count{0};
    uint16_t enable_attempts_{0};  // increment when we're in the STATUS_ENABLING state
    bool idle_{false};
};

#endif
