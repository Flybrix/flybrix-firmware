/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "PIDOption.h"

PIDOption::PIDOption(const PIDSettings& terms, bool use)
        : use{use},
          pid{terms},
          command_to_value{terms.command_to_value} {}

void PIDOption::update(ClockTime now, float& value) {
    if (use) {
        pid.setDesiredSetpoint(value);
        value = pid.Compute(now);
    } else {
        pid.setTimer(now);
    }
}

void PIDOption::integralReset() {
    pid.IntegralReset();
}

void PIDOption::setTimer(ClockTime now) {
    pid.setTimer(now);
}
