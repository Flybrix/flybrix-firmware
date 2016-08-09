/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <control.h/cpp>

    Takes pilot commands and state data and generates instantaneous control vectors that are passed over to airframe.

*/

#ifndef control_h
#define control_h

#include "Arduino.h"
#include "cascadedPID.h"

class PID;
class CONFIG_struct;
class State;

class Control {
   public:
    struct PIDParameters;

    Control(State* state, const PIDParameters& config);
    void parseConfig(const PIDParameters& config);

    void calculateControlVectors();

    State* state;

    struct __attribute__((packed)) PIDParameters {
        bool verify() const;

        float thrust_master[7];  // parameters are {P,I,D,integral windup guard, D filter delay sec, setpoint filter delay sec, command scaling factor}
        float pitch_master[7];
        float roll_master[7];
        float yaw_master[7];

        float thrust_slave[7];
        float pitch_slave[7];
        float roll_slave[7];
        float yaw_slave[7];

        uint8_t pid_bypass;  // bitfield order for bypass: {thrustMaster, pitchMaster, rollMaster, yawMaster, thrustSlave, pitchSlave, rollSlave, yawSlave} (LSB-->MSB)
    } pid_parameters;

    static_assert(sizeof(PIDParameters) == 4 * 8 * 7 + 1, "Data is not packed");

    uint32_t lastUpdateMicros = 0;  // 1.2 hrs should be enough

    // unpack config.pidBypass for convenience
    bool pidEnabled[8]{false, false, false, false, false, false, false, false};

    // controllers
    CascadedPID thrust_pid, pitch_pid, roll_pid, yaw_pid;
};

#endif
