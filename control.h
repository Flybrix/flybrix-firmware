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
#include "PIDCascade.h"
#include "controlVectors.h"
#include "utility/vector3.h"

class Kinematics;
struct RcCommand;

class Control {
   public:
    struct PIDParameters;
    struct VelocityPIDParameters;

    Control(const PIDParameters& config, const VelocityPIDParameters& velocity_config);
    void parseConfig();

    ControlVectors calculateControlVectors(const Vector3<float>& velocity, const Kinematics& feedback, const RcCommand& setpoint);

    struct __attribute__((packed)) PIDParameters {
        PIDParameters();
        bool verify() const;

        float thrust_master[7];  // parameters are {P,I,D,integral windup guard, D filter delay sec, setpoint filter delay sec, command scaling factor}
        float pitch_master[7];
        float roll_master[7];
        float yaw_master[7];

        float thrust_slave[7];
        float pitch_slave[7];
        float roll_slave[7];
        float yaw_slave[7];

        float thrust_gain;
        float pitch_gain;
        float roll_gain;
        float yaw_gain;

        uint8_t pid_bypass;  // bitfield order for bypass: {thrustMaster, pitchMaster, rollMaster, yawMaster, thrustSlave, pitchSlave, rollSlave, yawSlave} (LSB-->MSB)
    } pid_parameters;

    static_assert(sizeof(PIDParameters) == 4 * 8 * 7 + 4 * 4 + 1, "Data is not packed");

    struct __attribute__((packed)) VelocityPIDParameters {
        VelocityPIDParameters();
        bool verify() const;

        float forward_master[7];  // parameters are {P,I,D,integral windup guard, D filter delay sec, setpoint filter delay sec, command scaling factor}
        float right_master[7];
        float up_master[7];

        float forward_slave[7];
        float right_slave[7];
        float up_slave[7];

        uint8_t pid_bypass;  // bitfield order for bypass: {xMaster, yMaster, zMaster, -, xSlave, ySlave, zSlave, -} (LSB-->MSB)
    } velocity_pid_parameters;

    static_assert(sizeof(VelocityPIDParameters) == 3 * 8 * 7 + 1, "Data is not packed");

    uint32_t lastUpdateMicros = 0;  // 1.2 hrs should be enough

    // unpack config.pidBypass for convenience
    bool pidEnabled[8]{false, false, false, false, false, false, false, false};

    // controllers
    PIDCascade<4> forward_pid, right_pid, up_pid;
    PIDCascade<2> yaw_pid;
};

#endif
