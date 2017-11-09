/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <control.h/cpp>

    Takes pilot commands and state data and generates instantaneous control vectors that are passed over to airframe.

*/

#ifndef control_h
#define control_h

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

        PIDSettings thrust_master;
        PIDSettings pitch_master;
        PIDSettings roll_master;
        PIDSettings yaw_master;

        PIDSettings thrust_slave;
        PIDSettings pitch_slave;
        PIDSettings roll_slave;
        PIDSettings yaw_slave;

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

        PIDSettings forward_master;
        PIDSettings right_master;
        PIDSettings up_master;

        PIDSettings forward_slave;
        PIDSettings right_slave;
        PIDSettings up_slave;

        uint8_t pid_bypass;  // bitfield order for bypass: {xMaster, yMaster, zMaster, -, xSlave, ySlave, zSlave, -} (LSB-->MSB)
    } velocity_pid_parameters;

    static_assert(sizeof(VelocityPIDParameters) == 3 * 8 * 7 + 1, "Data is not packed");

    // controllers
    PIDCascade<4> forward_pid, right_pid, up_pid;
    PIDCascade<2> yaw_pid;

    bool bidirectional_throttle{false};
};

#endif
