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
#define BYPASS_FORWARD_MASTER 1 << 0
#define BYPASS_RIGHT_MASTER 1 << 1
#define BYPASS_UP_MASTER 1 << 2
#define BYPASS_FORWARD_SLAVE 1 << 4
#define BYPASS_RIGHT_SLAVE 1 << 5
#define BYPASS_UP_SLAVE 1 << 6

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

Control::VelocityPIDParameters::VelocityPIDParameters()
    : forward_master{1.0, 0.0, 0.0, 0.0, 0.005, 0.005, 1.0},
      right_master{10.0, 1.0, 0.0, 10.0, 0.005, 0.005, 10.0},
      up_master{10.0, 1.0, 0.0, 10.0, 0.005, 0.005, 10.0},
      forward_slave{1.0, 0.0, 0.0, 10.0, 0.001, 0.001, 0.3},
      right_slave{10.0, 4.0, 0.0, 30.0, 0.001, 0.001, 30.0},
      up_slave{10.0, 4.0, 0.0, 30.0, 0.001, 0.001, 30.0},
      pid_bypass{BYPASS_FORWARD_MASTER | BYPASS_RIGHT_MASTER | BYPASS_UP_MASTER | BYPASS_FORWARD_SLAVE | BYPASS_RIGHT_SLAVE | BYPASS_UP_SLAVE}  // Disabled
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

Control::Control(const PIDParameters& config, const VelocityPIDParameters& velocity_config)
    : pid_parameters(config),
      velocity_pid_parameters(velocity_config),
      forward_pid(velocity_config.forward_master, velocity_config.forward_slave, config.pitch_master, config.pitch_slave),
      right_pid(velocity_config.right_master, velocity_config.right_slave, config.roll_master, config.roll_slave),
      up_pid(velocity_config.up_master, velocity_config.up_slave, config.thrust_master, config.thrust_slave),
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

bool Control::VelocityPIDParameters::verify() const {
    return true;
}

void Control::parseConfig() {
    forward_pid = PIDCascade<4>(velocity_pid_parameters.forward_master, velocity_pid_parameters.forward_slave, pid_parameters.pitch_master, pid_parameters.pitch_slave);
    right_pid = PIDCascade<4>(velocity_pid_parameters.right_master, velocity_pid_parameters.right_slave, pid_parameters.roll_master, pid_parameters.roll_slave);
    up_pid = PIDCascade<4>(velocity_pid_parameters.up_master, velocity_pid_parameters.up_slave, pid_parameters.thrust_master, pid_parameters.thrust_slave);
    yaw_pid = PIDCascade<2>(pid_parameters.yaw_master, pid_parameters.yaw_slave);

    forward_pid.use<0>(((velocity_pid_parameters.pid_bypass & 0x1) == 0));
    forward_pid.use<1>(((velocity_pid_parameters.pid_bypass & 0x10) == 0));
    forward_pid.use<2>(((pid_parameters.pid_bypass & 0x2) == 0));
    forward_pid.use<3>(((pid_parameters.pid_bypass & 0x20) == 0));

    right_pid.use<0>(((velocity_pid_parameters.pid_bypass & 0x2) == 0));
    right_pid.use<1>(((velocity_pid_parameters.pid_bypass & 0x20) == 0));
    right_pid.use<2>(((pid_parameters.pid_bypass & 0x4) == 0));
    right_pid.use<3>(((pid_parameters.pid_bypass & 0x40) == 0));

    up_pid.use<0>(((velocity_pid_parameters.pid_bypass & 0x4) == 0));
    up_pid.use<1>(((velocity_pid_parameters.pid_bypass & 0x40) == 0));
    up_pid.use<2>(((pid_parameters.pid_bypass & 0x1) == 0));
    up_pid.use<3>(((pid_parameters.pid_bypass & 0x10) == 0));

    yaw_pid.use<0>(((pid_parameters.pid_bypass & 0x8) == 0));
    yaw_pid.use<1>(((pid_parameters.pid_bypass & 0x80) == 0));

    // all slaves are rate controllers; set up the master pids as wrapped angle controllers
    forward_pid.pid<2>().isWrapped();
    right_pid.pid<2>().isWrapped();
    yaw_pid.pid<0>().isWrapped();

    up_pid.integralReset();
    forward_pid.integralReset();
    right_pid.integralReset();
    yaw_pid.integralReset();

    up_pid.setDefaultScaling(pid_parameters.thrust_gain);
    forward_pid.setDefaultScaling(pid_parameters.pitch_gain);
    right_pid.setDefaultScaling(pid_parameters.roll_gain);
    yaw_pid.setDefaultScaling(pid_parameters.yaw_gain);
}

float radToDeg(float v) {
    return v * 57.2957795f;
}

ControlVectors Control::calculateControlVectors(const Vector3<float>& velocity, const Kinematics& feedback, const RcCommand& setpoint) {
    up_pid.pid<2>().setInput(feedback.altitude);
    up_pid.pid<3>().setInput(0.0f);  // feedback.ClimbRate
    forward_pid.pid<2>().setInput(radToDeg(feedback.angle.pitch));
    forward_pid.pid<3>().setInput(radToDeg(feedback.rate.pitch));
    right_pid.pid<2>().setInput(radToDeg(feedback.angle.roll));
    right_pid.pid<3>().setInput(radToDeg(feedback.rate.roll));
    yaw_pid.pid<0>().setInput(radToDeg(feedback.angle.yaw));
    yaw_pid.pid<1>().setInput(radToDeg(feedback.rate.yaw));

    forward_pid.pid<1>().setInput(velocity.x);
    right_pid.pid<1>().setInput(velocity.y);
    up_pid.pid<1>().setInput(velocity.z);

    // TODO: adjust setpoint based on bypass flags
    up_pid.setSetpoint(setpoint.throttle * (1.0f / 4095.0f) * up_pid.getScalingFactor());
    forward_pid.setSetpoint(setpoint.pitch * (1.0f / 2047.0f) * forward_pid.getScalingFactor());
    right_pid.setSetpoint(setpoint.roll * (1.0f / 2047.0f) * right_pid.getScalingFactor());
    yaw_pid.setSetpoint(setpoint.yaw * (1.0f / 2047.0f) * yaw_pid.getScalingFactor());

    // compute new output levels for state
    uint32_t now = micros();

    ControlVectors control;
    control.force_z = up_pid.compute(now);
    control.torque_x = forward_pid.compute(now);
    control.torque_y = right_pid.compute(now);
    control.torque_z = yaw_pid.compute(now);

    if (control.force_z == 0) {  // throttle is in low condition
        control.torque_x = 0;
        control.torque_y = 0;
        control.torque_z = 0;

        up_pid.integralReset();
        forward_pid.integralReset();
        right_pid.integralReset();
        yaw_pid.integralReset();
    }

    return control;
}
