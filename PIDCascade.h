/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef PID_CASCADE_H
#define PID_CASCADE_H

#include <array>

#include "PIDOption.h"
#include "PIDSettings.h"
#include "ClockTime.h"

template<size_t N>
class PIDCascade final {
public:
    template<typename... Targs>
    explicit PIDCascade(Targs... terms)
            : regulators_{terms...} {}

    template<size_t M>
    const PID& pid() const {
        return std::get<M>(regulators_).pid;
    }

    template<size_t M>
    PID& pid() {
        return std::get<M>(regulators_).pid;
    }

    template<size_t M>
    void use(bool should_use) {
        std::get<M>(regulators_).use = should_use;
    }

    float getScalingFactor() const;
    void setSetpoint(float v);
    void setDefaultScaling(float v);
    void integralReset();
    void timerReset(ClockTime now);
    float compute(ClockTime now);

private:
    std::array <PIDOption, N> regulators_;
    float setpoint_{0.0f};
    float default_scaling_{1.0f};
};

#include "PIDCascade_impl.h"

#endif  // PID_CASCADE_H
