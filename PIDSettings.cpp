/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "PIDSettings.h"

PIDSettings::PIDSettings() : PIDSettings(0, 0, 0, 0, 0, 0, 0) {}

PIDSettings::PIDSettings(float kp, float ki, float kd, float integral_windup_guard, float d_filter_time,
                         float setpoint_filter_time, float command_to_value)
        : kp{kp},
          ki{ki},
          kd{kd},
          integral_windup_guard{integral_windup_guard},
          d_filter_time{d_filter_time},
          setpoint_filter_time{setpoint_filter_time},
          command_to_value{command_to_value} {}

bool PIDSettings::verify() const {
    return integral_windup_guard >= 0.0 && d_filter_time >= 0.0 && setpoint_filter_time >= 0.0;
}
