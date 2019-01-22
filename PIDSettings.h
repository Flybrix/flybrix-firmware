/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef PID_SETTINGS_H
#define PID_SETTINGS_H

struct __attribute__((packed)) PIDSettings {
    PIDSettings();
    PIDSettings(float kp, float ki, float kd, float integral_windup_guard, float d_filter_time,
                float setpoint_filter_time, float command_to_value);
    bool verify() const;

    float kp;
    float ki;
    float kd;
    float integral_windup_guard;
    float d_filter_time;
    float setpoint_filter_time;
    float command_to_value;
};

static_assert(sizeof(PIDSettings) == 7 * 4, "Data is not packed");

#endif // PID_SETTINGS_H
