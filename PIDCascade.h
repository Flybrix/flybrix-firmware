/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef PID_CASCADE_H
#define PID_CASCADE_H

#include <array>

#include "PID.h"

template <size_t N>
class PIDCascade final {
   public:
    template <typename... Targs>
    explicit PIDCascade(Targs... terms) : regulators_{terms...} {
    }

    template <size_t M>
    const PID& pid() const {
        return std::get<M>(regulators_).pid;
    }

    template <size_t M>
    PID& pid() {
        return std::get<M>(regulators_).pid;
    }

    template <size_t M>
    void use(bool should_use) {
        std::get<M>(regulators_).use = should_use;
    }

    float getScalingFactor() const {
        for (const Regulator& reg : regulators_) {
            if (reg.use) {
                return reg.pid.commandToValue();
            }
        }
        return default_scaling_;
    }

    void setSetpoint(float v) {
        setpoint_ = v;
    }

    void setDefaultScaling(float v) {
        default_scaling_ = v;
    }

    void integralReset() {
        for (Regulator& reg : regulators_) {
            reg.pid.IntegralReset();
        }
    }

    void timerReset(uint32_t now) {
        for (Regulator& reg : regulators_) {
            reg.pid.setTimer(now);
        }
    }

    float compute(uint32_t now) {
        float value{setpoint_};
        for (Regulator& reg : regulators_) {
            if (reg.use) {
                reg.pid.setSetpoint(value);
                value = reg.pid.Compute(now);
            } else {
                reg.pid.setTimer(now);
            }
        }
        return value;
    }

   private:
    struct Regulator {
        Regulator(const PIDSettings& terms) : use{false}, pid{terms} {
        }

        bool use;
        PID pid;
    };

    std::array<Regulator, N> regulators_;
    float setpoint_{0.0f};
    float default_scaling_{1.0f};
};

#endif  // PID_CASCADE_H
