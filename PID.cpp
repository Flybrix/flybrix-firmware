/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "PID.h"

PID::PID(const PIDSettings& settings)
        : Kp{settings.kp},
          Ki{settings.ki},
          Kd{settings.kd},
          integral_windup_guard{settings.integral_windup_guard},
          d_filter{0.0f, settings.d_filter_time},
          setpoint_filter{0.0f, settings.setpoint_filter_time},
          command_to_value{settings.command_to_value} {}

ClockTime PID::lastTime() const {
    return last_time;
}

float PID::p() const {
    return p_term;
}

float PID::i() const {
    return i_term;
}

float PID::d() const {
    return d_term;
}

float PID::input() const {
    return input_;
}

float PID::setpoint() const {
    return setpoint_;
}

float PID::desiredSetpoint() const {
    return desired_setpoint_;
}

float PID::commandToValue() const {
    return command_to_value;
}

void PID::setWrapped(bool wrapped) {
    degrees = wrapped;
}

void PID::setInput(float v) {
    input_ = v;
}

void PID::setSetpoint(float v) {
    desired_setpoint_ = v;
}

void PID::setTimer(ClockTime now) {
    last_time = now;
}

float PID::Compute(ClockTime now) {
    float delta_time = (now - last_time) / 1000000.0;

    setpoint_ = setpoint_filter.update(desired_setpoint_, delta_time);

    float error = setpoint_ - input_;

    if (degrees) {
        while (error < -180.0f) {
            error += 360.0f;
        }
        while (error >= 180.0f) {
            error -= 360.0f;
        }
    }

    p_term = Kp * error;

    i_term = Ki * error_integral;

    error_integral += error * delta_time;

    float windup_limit = integral_windup_guard / Ki;
    if (error_integral > windup_limit) {
        error_integral = windup_limit;
    } else if (error_integral < -windup_limit) {
        error_integral = -windup_limit;
    }

    d_term = d_filter.update(Kd * ((error - previous_error) / delta_time), delta_time);

    previous_error = error;
    last_time = now;

    return p_term + i_term + d_term;
}

void PID::IntegralReset() {
    error_integral = 0.0f;
}
