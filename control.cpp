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
        nonzero_input_[i] = ((config.pidBypass & (1 << i)) == 0);

    // yaw controls a rate, not an angle!
    // all slaves are rate controllers
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
    thrust_pid.setSlaveInput(0.0f);  // TODO: where would be get this data from?
    pitch_pid.setMasterInput(state->kinematicsAngle[0] * 57.2957795);
    pitch_pid.setSlaveInput(state->kinematicsRate[0] * 57.2957795);
    roll_pid.setMasterInput(state->kinematicsAngle[1] * 57.2957795);
    roll_pid.setSlaveInput(state->kinematicsRate[1] * 57.2957795);
    yaw_pid.setMasterInput(state->kinematicsAngle[2] * 57.2957795);
    yaw_pid.setSlaveInput(state->kinematicsRate[2] * 57.2957795);

    thrust_pid.setSetpoint(state->command_throttle * thrust_pid.getScalingFactor(nonzero_input_[THRUST_MASTER], nonzero_input_[THRUST_SLAVE]));
    pitch_pid.setSetpoint(state->command_pitch * pitch_pid.getScalingFactor(nonzero_input_[PITCH_MASTER], nonzero_input_[PITCH_SLAVE]));
    roll_pid.setSetpoint(state->command_roll * roll_pid.getScalingFactor(nonzero_input_[ROLL_MASTER], nonzero_input_[ROLL_SLAVE]));
    yaw_pid.setSetpoint(state->command_yaw * yaw_pid.getScalingFactor(nonzero_input_[YAW_MASTER], nonzero_input_[YAW_SLAVE]));

    uint32_t now = micros();

    // compute new slave setpoints in the master PIDs
    // and compute state control torques
    state->Fz = thrust_pid.Compute(now, nonzero_input_[THRUST_MASTER], nonzero_input_[THRUST_SLAVE]);
    state->Tx = pitch_pid.Compute(now, nonzero_input_[PITCH_MASTER], nonzero_input_[PITCH_SLAVE]);
    state->Ty = roll_pid.Compute(now, nonzero_input_[ROLL_MASTER], nonzero_input_[ROLL_SLAVE]);
    state->Tz = yaw_pid.Compute(now, nonzero_input_[YAW_MASTER], nonzero_input_[YAW_SLAVE]);

    // add in the feedforward torques
    state->Tx += state->Tx_trim;
    state->Ty += state->Ty_trim;
    state->Tz += state->Tz_trim;

    // use the updated pid outputs to calculate the motor drive terms
    if (state->Fz == 0) {  // throttle is in low condition
        state->Tx = 0;
        state->Ty = 0;
        state->Tz = 0;
    }
}
