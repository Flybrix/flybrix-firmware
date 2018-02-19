/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef command_h
#define command_h

#include <Arduino.h>

#include "airframe.h"
#include "ledDriver.h"
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

    bool isArmingFailureState() const;
    LEDPattern::Pattern failToPattern() const;
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
    bool idle_{false};
};

#endif
