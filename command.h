/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef command_h
#define command_h

#include <Arduino.h>

#include "airframe.h"
#include "utility/rcHelpers.h"
#include "utility/ticker.h"

struct Systems;
struct CommandVector;
struct ControlVectors;
class BMP280;
class State;
class StateFlag;
class Imu;
class LED;

class PilotCommand {
   public:
    explicit PilotCommand(Systems& systems);
    RcCommand processCommands(RcState&&);
    // Applies freshly received stick values to `current` (the last processCommands() result) without waiting for
    // the next 40Hz processCommands() call -- but only while that call is passing the sticks through unchanged
    // (armed, flying, radio link up). Otherwise `current` is returned as is. Arm switches are never taken from here.
    RcCommand refreshSticks(const RcCommand& fresh, const RcCommand& current) const;
    void processMotorEnablingIteration();
    void disableMotors();
    // Cut the motors now (e.g. the attitude data has gone stale). The pilot must cycle the arm switch to re-arm.
    void emergencyDisarm();
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

    void cutMotors();  // disable the motors without changing the control state
    void startFailsafe();
    void endFailsafe(const RcCommand& command);
    RcCommand failsafeCommand();
    void setControlState(ControlState state);

    bool isArmingFailureState() const;
    uint32_t failToColor() const;

    bool upright() const;
    bool stable() const;

    BMP280& bmp_;
    State& state_;
    Imu& imu_;
    StateFlag& flag_;
    LED& led_;

    Airframe airframe_;
    ControlState control_state_{ControlState::AwaitingAuxDisable};
    Ticker<uint8_t> throttle_hold_off_;  // hold controls low for some time after enabling
    int16_t invalid_count{0};
    uint16_t enable_attempts_{0};  // increment when we're in the STATUS_ARMING state
    RcCommand last_command_;         // last command passed on while the radio link was up
    float failsafe_throttle_{0.0f};  // throttle commanded during an RX-loss descent
    uint16_t failsafe_ticks_{0};
    bool sticks_pass_through_{false};  // the last processCommands() passed the stick values through unchanged
    bool idle_{false};
};

#endif
