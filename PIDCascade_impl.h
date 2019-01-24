/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef PID_CASCADE_IMPL_H
#define PID_CASCADE_IMPL_H

#include "PIDCascade.h"

template<size_t N>
float PIDCascade<N>::getScalingFactor() const {
    for (const PIDOption& reg : regulators_) {
        if (reg.use) {
            return reg.command_to_value;
        }
    }
    return default_scaling_;
}

template<size_t N>
void PIDCascade<N>::setSetpoint(float v) {
    setpoint_ = v;
}

template<size_t N>
void PIDCascade<N>::setDefaultScaling(float v) {
    default_scaling_ = v;
}

template<size_t N>
void PIDCascade<N>::integralReset() {
    for (PIDOption& reg : regulators_) {
        reg.integralReset();
    }
}

template<size_t N>
void PIDCascade<N>::timerReset(ClockTime now) {
    for (PIDOption& reg : regulators_) {
        reg.setTimer(now);
    }
}

template<size_t N>
float PIDCascade<N>::compute(ClockTime now) {
    float value{setpoint_};
    for (PIDOption& reg : regulators_) {
        reg.update(now, value);
    }
    return value;
}

#endif  // PID_CASCADE_IMPL_H
