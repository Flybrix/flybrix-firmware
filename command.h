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

struct Systems;
struct CommandVector;
struct ControlVectors;
class State;
class StateFlag;
class MPU9250;
class R415X;

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

   private:
    class Ticker final {
       public:
        bool tick();
        void reset(uint8_t ticks);

       private:
        uint8_t count_{0};
    };

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
    MPU9250& mpu_;
    R415X& receiver_;
    StateFlag& flag_;
    CommandVector& command_vector_;

    Airframe airframe_;
    ControlState control_state_{ControlState::AwaitingAuxDisable};
    Ticker throttle_hold_off_;  // hold controls low for some time after enabling
    Ticker bluetooth_tolerance_;
    int16_t invalid_count{0};
    uint16_t enable_attempts_{0};  // increment when we're in the STATUS_ENABLING state
    bool idle_{false};
};

#endif
