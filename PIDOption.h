/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef PID_OPTION_H
#define PID_OPTION_H

#include "ClockTime.h"
#include "PID.h"
#include "PIDSettings.h"

struct PIDOption {
    PIDOption(const PIDSettings& terms, bool use = false);

    void update(ClockTime now, float& value);
    void integralReset();
    void setTimer(ClockTime now);

    bool use;
    PID pid;
    float command_to_value;
};


#endif // PID_OPTION_H
