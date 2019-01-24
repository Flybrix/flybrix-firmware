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
          windup_limit{settings.integral_windup_guard / settings.ki},
          d_filter{0.0f, settings.d_filter_time},
          setpoint_filter{0.0f, settings.setpoint_filter_time},
          command_to_value{settings.command_to_value} {}

ClockTime PID::lastTime() const {
    return last_time;
}

float PID::pTerm() const {
    return p_term;
}

float PID::iTerm() const {
    return i_term;
}

float PID::dTerm() const {
    return d_term;
}

float PID::input() const {
    return input_;
}

float PID::filteredSetpoint() const {
    return filtered_setpoint_;
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

void PID::setDesiredSetpoint(float v) {
    desired_setpoint_ = v;
}

void PID::setTimer(ClockTime now) {
    last_time = now;
}

float PID::wrapDegrees(float value) const {
    if (!degrees) {
        return value;
    }
    while (value < -180.0f) {
        value += 360.0f;
    }
    while (value >= 180.0f) {
        value -= 360.0f;
    }
    return value;
}

float PID::limitWindup(float value) const {
    if (value > windup_limit) {
        return windup_limit;
    }
    if (value < -windup_limit) {
        return -windup_limit;
    }
    return value;
}

float PID::computeDeltaTime(ClockTime now) {
    float delta_time = (now - last_time) / 1000000.0;
    last_time = now;
    return delta_time;
}

void PID::computeFilteredSetpoint(float dt) {
    filtered_setpoint_ = setpoint_filter.update(desired_setpoint_, dt);
}

void PID::computeError(float dt) {
    float new_error = wrapDegrees(filtered_setpoint_ - input_);

    error_derivative_ = wrapDegrees(new_error - error_) / dt;
    error_ = new_error;
    error_integral_ = limitWindup(error_integral_ + error_ * dt);
}

void PID::computeTerms(float dt) {
    p_term = Kp * error_;
    i_term = Ki * error_integral_;
    d_term = d_filter.update(Kd * error_derivative_, dt);
}

float PID::Compute(ClockTime now) {
    float delta_time = computeDeltaTime(now);

    computeFilteredSetpoint(delta_time);
    computeError(delta_time);
    computeTerms(delta_time);

    return p_term + i_term + d_term;
}

void PID::IntegralReset() {
    error_integral_ = 0.0f;
}
