/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
    *

    <PID.h>

*/

#ifndef PID_h
#define PID_h

#include <cstdint>

class IIRfilter {
   public:
    IIRfilter(float _output, float _time_constant) {
        out = _output;
        tau = _time_constant;
    };
    float update(float in, float dt) {
        return out = (in * dt + out * tau) / (dt + tau);
    };

   private:
    float out;
    float tau;
};

class PID {
   public:
    explicit PID(const float* terms)
        : Kp{terms[0]}, Ki{terms[1]}, Kd{terms[2]}, integral_windup_guard{terms[3]}, d_filter{0.0f, terms[4]}, setpoint_filter{0.0f, terms[5]}, command_to_value{terms[6]} {};

    void isWrapped(bool wrapped = true) {
        degrees = wrapped;
    }

    uint32_t lastTime() const {
        return last_time;
    }

    float pTerm() const {
        return p_term;
    }

    float iTerm() const {
        return i_term;
    }

    float dTerm() const {
        return d_term;
    }

    float input() const {
        return input_;
    }

    float setpoint() const {
        return setpoint_;
    }

    float desiredSetpoint() const {
        return desired_setpoint_;
    }

    float commandToValue() const {
        return command_to_value;
    }

    void setInput(float v) {
        input_ = v;
    }

    void setSetpoint(float v) {
        desired_setpoint_ = v;
    }

    void setTimer(uint32_t now) {
        last_time = now;
    }

    float Compute(uint32_t now) {
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
    };

    void IntegralReset() {
        error_integral = 0.0f;
        desired_setpoint_ = 0.0f;
    };

   private:
    float Kp;
    float Ki;
    float Kd;
    float integral_windup_guard;
    IIRfilter d_filter;
    IIRfilter setpoint_filter;
    float command_to_value;

    float input_{0.0f}, setpoint_{0.0f};
    float desired_setpoint_{0.0f};

    uint32_t last_time{0};
    float p_term{0.0f};
    float i_term{0.0f};
    float d_term{0.0f};

    bool degrees{false};  // unwrap error terms for angle control in degrees

    float previous_error{0.0f};
    float error_integral{0.0f};
};

#endif
