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
    float pTerm() const;
    float iTerm() const;
    float dTerm() const;
    float input() const;
    float filteredSetpoint() const;
    float desiredSetpoint() const;

    void setWrapped(bool wrapped = true);
    void setInput(float v);
    void setDesiredSetpoint(float v);
    void setTimer(ClockTime now);
    float Compute(ClockTime now);
    void IntegralReset();

private:
    float wrapDegrees(float value) const;
    float limitWindup(float value) const;

    float computeDeltaTime(ClockTime now);
    void computeFilteredSetpoint(float dt);
    void computeError(float dt);
    void computeTerms(float dt);

    float Kp;
    float Ki;
    float Kd;
    float integral_windup_guard;
    float windup_limit;

    IIRFilter d_filter;
    IIRFilter setpoint_filter;

    float input_{0.0f};
    float desired_setpoint_{0.0f};
    float filtered_setpoint_{0.0f};

    ClockTime last_time{ClockTime::zero()};
    float p_term{0.0f};
    float i_term{0.0f};
    float d_term{0.0f};

    bool degrees{false};  // unwrap error terms for angle control in degrees

    float error_{0.0f};
    float error_integral_{0.0f};
    float error_derivative_{0.0f};
};

#endif
