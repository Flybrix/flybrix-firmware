/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "control.h"
#include "debug.h"
#include "kinematics.h"
#include "utility/rcHelpers.h"

#define BYPASS_THRUST_MASTER 1 << 0
#define BYPASS_PITCH_MASTER 1 << 1
#define BYPASS_ROLL_MASTER 1 << 2
#define BYPASS_YAW_MASTER 1 << 3
#define BYPASS_THRUST_SLAVE 1 << 4
#define BYPASS_PITCH_SLAVE 1 << 5
#define BYPASS_ROLL_SLAVE 1 << 6
#define BYPASS_YAW_SLAVE 1 << 7

// PID parameters:

// MASTER:
// P
// I
// D
// Windup guard
// D filter usec (15Hz)
// setpoint filter usec (30Hz)
// (meters / full stick action)

// SLAVE:
// P
// I
// D
// Windup guard
// D filter usec (150Hz)
// setpoint filter usec (300Hz)
// (deg/sec / full stick action)

Control::PIDParameters::PIDParameters()
    : thrust_master{1.0, 0.0, 0.0, 0.0, 0.005, 0.005, 1.0},
      pitch_master{10.0, 1.0, 0.0, 10.0, 0.005, 0.005, 10.0},
      roll_master{10.0, 1.0, 0.0, 10.0, 0.005, 0.005, 10.0},
      yaw_master{5.0, 1.0, 0.0, 10.0, 0.005, 0.005, 180.0},
      thrust_slave{1.0, 0.0, 0.0, 10.0, 0.001, 0.001, 0.3},
      pitch_slave{10.0, 4.0, 0.0, 30.0, 0.001, 0.001, 30.0},
      roll_slave{10.0, 4.0, 0.0, 30.0, 0.001, 0.001, 30.0},
      yaw_slave{30.0, 5.0, 0.0, 20.0, 0.001, 0.001, 240.0},
      thrust_gain{4095.0},
      pitch_gain{2047.0},
      roll_gain{2047.0},
      yaw_gain{2047.0},
      pid_bypass{BYPASS_THRUST_MASTER | BYPASS_THRUST_SLAVE | BYPASS_YAW_MASTER}  // AHRS/Horizon mode
{
}

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

Control::Control(const PIDParameters& config)
    : pid_parameters(config),
      thrust_pid(config.thrust_master, config.thrust_slave),
      pitch_pid(config.pitch_master, config.pitch_slave),
      roll_pid(config.roll_master, config.roll_slave),
      yaw_pid(config.yaw_master, config.yaw_slave) {
    parseConfig();
}

bool Control::PIDParameters::verify() const {
    bool retval{true};
    if (!(pid_bypass & (1 << THRUST_SLAVE))) {
        // If the thrust slave is enabled
        DebugPrint("The slave PID of the thrust regulator must be disabled for now");
        retval = false;
    }
    return retval;
}

void Control::parseConfig() {
    thrust_pid = PIDCascade<2>(pid_parameters.thrust_master, pid_parameters.thrust_slave);
    pitch_pid = PIDCascade<2>(pid_parameters.pitch_master, pid_parameters.pitch_slave);
    roll_pid = PIDCascade<2>(pid_parameters.roll_master, pid_parameters.roll_slave);
    yaw_pid = PIDCascade<2>(pid_parameters.yaw_master, pid_parameters.yaw_slave);

    thrust_pid.use<0>(((pid_parameters.pid_bypass & 0x1) == 0));
    pitch_pid.use<0>(((pid_parameters.pid_bypass & 0x2) == 0));
    roll_pid.use<0>(((pid_parameters.pid_bypass & 0x4) == 0));
    yaw_pid.use<0>(((pid_parameters.pid_bypass & 0x8) == 0));
    thrust_pid.use<1>(((pid_parameters.pid_bypass & 0x10) == 0));
    pitch_pid.use<1>(((pid_parameters.pid_bypass & 0x20) == 0));
    roll_pid.use<1>(((pid_parameters.pid_bypass & 0x40) == 0));
    yaw_pid.use<1>(((pid_parameters.pid_bypass & 0x80) == 0));

    // all slaves are rate controllers; set up the master pids as wrapped angle controllers
    pitch_pid.pid<0>().isWrapped();
    roll_pid.pid<0>().isWrapped();
    yaw_pid.pid<0>().isWrapped();

    thrust_pid.integralReset();
    pitch_pid.integralReset();
    roll_pid.integralReset();
    yaw_pid.integralReset();

    thrust_pid.setDefaultScaling(pid_parameters.thrust_gain);
    pitch_pid.setDefaultScaling(pid_parameters.pitch_gain);
    roll_pid.setDefaultScaling(pid_parameters.roll_gain);
    yaw_pid.setDefaultScaling(pid_parameters.yaw_gain);
}

float radToDeg(float v) {
    return v * 57.2957795f;
}

ControlVectors Control::calculateControlVectors(const Kinematics& feedback, const RcCommand& setpoint) {
    thrust_pid.pid<0>().setInput(feedback.altitude);
    thrust_pid.pid<0>().setInput(0.0f);  // feedback.ClimbRate
    pitch_pid.pid<0>().setInput(radToDeg(feedback.angle.pitch));
    pitch_pid.pid<0>().setInput(radToDeg(feedback.rate.pitch));
    roll_pid.pid<0>().setInput(radToDeg(feedback.angle.roll));
    roll_pid.pid<0>().setInput(radToDeg(feedback.rate.roll));
    yaw_pid.pid<0>().setInput(radToDeg(feedback.angle.yaw));
    yaw_pid.pid<0>().setInput(radToDeg(feedback.rate.yaw));

    thrust_pid.setSetpoint(setpoint.throttle * (1.0f / 4095.0f) * thrust_pid.getScalingFactor());
    pitch_pid.setSetpoint(setpoint.pitch * (1.0f / 2047.0f) * pitch_pid.getScalingFactor());
    roll_pid.setSetpoint(setpoint.roll * (1.0f / 2047.0f) * roll_pid.getScalingFactor());
    yaw_pid.setSetpoint(setpoint.yaw * (1.0f / 2047.0f) * yaw_pid.getScalingFactor());

    // compute new output levels for state
    uint32_t now = micros();

    ControlVectors control;
    control.force_z = thrust_pid.compute(now);
    control.torque_x = pitch_pid.compute(now);
    control.torque_y = roll_pid.compute(now);
    control.torque_z = yaw_pid.compute(now);

    if (control.force_z == 0) {  // throttle is in low condition
        control.torque_x = 0;
        control.torque_y = 0;
        control.torque_z = 0;

        thrust_pid.integralReset();
        pitch_pid.integralReset();
        roll_pid.integralReset();
        yaw_pid.integralReset();
    }

    return control;
}
