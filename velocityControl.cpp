#include "velocityControl.h"

#include "utility/rcHelpers.h"

VelocityControl::PIDParameters::PIDParameters()
    : vx{10.0, 1.0, 0.0, 10.0, 0.005, 0.005, 10.0},
      vy{10.0, 1.0, 0.0, 10.0, 0.005, 0.005, 10.0},
      vz{10.0, 1.0, 0.0, 10.0, 0.005, 0.005, 10.0},
      bypass{true}  // default to not using
{
}

VelocityControl::VelocityControl(const PIDParameters& config) : pid_parameters(config), vx_pid{config.vx}, vy_pid{config.vy}, vz_pid{config.vz} {
    parseConfig();
}

void VelocityControl::parseConfig() {
    vx_pid = PID(pid_parameters.vx);
    vy_pid = PID(pid_parameters.vy);
    vz_pid = PID(pid_parameters.vz);
    bypass = pid_parameters.bypass;

    vx_pid.IntegralReset();
    vy_pid.IntegralReset();
    vz_pid.IntegralReset();
}

RcCommand VelocityControl::calculateControlVectors(const Vector3<float>& velocity, RcCommand setpoint) {
    uint32_t now = micros();
    if (bypass) {
        vx_pid.setTimer(now);
        vy_pid.setTimer(now);
        vz_pid.setTimer(now);
        return setpoint;
    }

    vx_pid.setInput(velocity.x);
    vy_pid.setInput(velocity.y);
    vz_pid.setInput(velocity.z);

    vx_pid.setSetpoint(setpoint.pitch * 1.0f / 2047.0f);
    vy_pid.setSetpoint(setpoint.roll * 1.0f / 2047.0f);
    vz_pid.setSetpoint((setpoint.throttle - 2048) * 1.0f / 2047.0f);

    setpoint.pitch = vx_pid.Compute(now);
    setpoint.roll = vy_pid.Compute(now);
    setpoint.yaw = 0;
    setpoint.throttle = vz_pid.Compute(now);

    return setpoint;
}
