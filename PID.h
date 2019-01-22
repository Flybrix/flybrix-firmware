/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef PID_h
#define PID_h

#include <cstdint>
#include "ClockTime.h"
#include "IIRFilter.h"
#include "PIDSettings.h"

class PID {
public:
    explicit PID(const PIDSettings& settings);

    ClockTime lastTime() const;
    float p() const;
    float i() const;
    float d() const;
    float input() const;
    float setpoint() const;
    float desiredSetpoint() const;
    float commandToValue() const;

    void setWrapped(bool wrapped = true);
    void setInput(float v);
    void setSetpoint(float v);
    void setTimer(ClockTime now);
    float Compute(ClockTime now);
    void IntegralReset();

private:
    float Kp;
    float Ki;
    float Kd;
    float integral_windup_guard;
    IIRFilter d_filter;
    IIRFilter setpoint_filter;
    float command_to_value;

    float input_{0.0f}, setpoint_{0.0f};
    float desired_setpoint_{0.0f};

    ClockTime last_time{ClockTime::zero()};
    float p_term{0.0f};
    float i_term{0.0f};
    float d_term{0.0f};

    bool degrees{false};  // unwrap error terms for angle control in degrees

    float previous_error{0.0f};
    float error_integral{0.0f};
};

#endif
