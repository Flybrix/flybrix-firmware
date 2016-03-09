/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "control.h"
#include "config.h"
#include "state.h"

namespace {
enum PID_ID {
    THRUST_MASTER = 0,
    PITCH_MASTER = 1,
    ROLL_MASTER = 2,
    YAW_MASTER = 3,
    THRUST_SLAVE = 4,
    PITCH_SLAVE = 5,
    ROLL_SLAVE = 6,
    YAW_SLAVE = 7,
};
}

Control::Control(State* __state, CONFIG_struct& config)
    : state(__state),
      thrust_pid{config.thrustMasterPIDParameters, config.thrustSlavePIDParameters},
      pitch_pid{config.pitchMasterPIDParameters, config.pitchSlavePIDParameters},
      roll_pid{config.rollMasterPIDParameters, config.rollSlavePIDParameters},
      yaw_pid{config.yawMasterPIDParameters, config.yawSlavePIDParameters} {
    parseConfig(config);
}

void Control::parseConfig(CONFIG_struct& config) {
    thrust_pid = {config.thrustMasterPIDParameters, config.thrustSlavePIDParameters};
    pitch_pid = {config.pitchMasterPIDParameters, config.pitchSlavePIDParameters};
    roll_pid = {config.rollMasterPIDParameters, config.rollSlavePIDParameters};
    yaw_pid = {config.yawMasterPIDParameters, config.yawSlavePIDParameters};

    for (uint8_t i = 0; i < 8; ++i)
        pidEnabled[i] = ((config.pidBypass & (1 << i)) == 0);

    // all slaves are rate controllers; set up the master pids as wrapped angle controllers
    pitch_pid.isMasterWrapped();
    roll_pid.isMasterWrapped();
    yaw_pid.isMasterWrapped();

    thrust_pid.IntegralReset();
    pitch_pid.IntegralReset();
    roll_pid.IntegralReset();
    yaw_pid.IntegralReset();
}

void Control::calculateControlVectors() {
    thrust_pid.setMasterInput(state->kinematicsAltitude);
    thrust_pid.setSlaveInput(0.0f); //state->kinematicsClimbRate
    pitch_pid.setMasterInput(state->kinematicsAngle[0] * 57.2957795);
    pitch_pid.setSlaveInput(state->kinematicsRate[0] * 57.2957795);
    roll_pid.setMasterInput(state->kinematicsAngle[1] * 57.2957795);
    roll_pid.setSlaveInput(state->kinematicsRate[1] * 57.2957795);
    yaw_pid.setMasterInput(state->kinematicsAngle[2] * 57.2957795);
    yaw_pid.setSlaveInput(state->kinematicsRate[2] * 57.2957795);

    thrust_pid.setSetpoint(state->command_throttle * 4095.0f * thrust_pid.getScalingFactor(pidEnabled[THRUST_MASTER], pidEnabled[THRUST_SLAVE]));
    pitch_pid.setSetpoint(state->command_pitch * 2047.0f * pitch_pid.getScalingFactor(pidEnabled[PITCH_MASTER], pidEnabled[PITCH_SLAVE]));
    roll_pid.setSetpoint(state->command_roll * 2047.0f * roll_pid.getScalingFactor(pidEnabled[ROLL_MASTER], pidEnabled[ROLL_SLAVE]));
    yaw_pid.setSetpoint(state->command_yaw * 2047.0f * yaw_pid.getScalingFactor(pidEnabled[YAW_MASTER], pidEnabled[YAW_SLAVE]));

    // compute new output levels for state
    uint32_t now = micros();
    state->Fz = thrust_pid.Compute(now, pidEnabled[THRUST_MASTER], pidEnabled[THRUST_SLAVE]);
    state->Tx = pitch_pid.Compute(now, pidEnabled[PITCH_MASTER], pidEnabled[PITCH_SLAVE]);
    state->Ty = roll_pid.Compute(now, pidEnabled[ROLL_MASTER], pidEnabled[ROLL_SLAVE]);
    state->Tz = yaw_pid.Compute(now, pidEnabled[YAW_MASTER], pidEnabled[YAW_SLAVE]);

    if (state->Fz == 0) {  // throttle is in low condition
        state->Tx = 0;
        state->Ty = 0;
        state->Tz = 0;

        thrust_pid.IntegralReset();
        pitch_pid.IntegralReset();
        roll_pid.IntegralReset();
        yaw_pid.IntegralReset();
    }
}
