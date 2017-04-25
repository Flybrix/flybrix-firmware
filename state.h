/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <state.h/cpp>

    State tracks everything we know about the UAV, computes derived state variables, and handles data logging

*/

#ifndef state_h
#define state_h

#include <cstdint>
#include "localization.h"
#include "utility/vector3.h"

struct Systems;

class State {
   public:
    explicit State(Systems* sys);

    // timing
    uint32_t loopCount = 0;

    // MPU9250
    float accel_filter[3] = {0.0, 0.0, 0.0}, accel_filter_sq[3] = {0.0, 0.0, 0.0};  // for stability variance calculation
    float gyro_filter[3] = {0.0, 0.0, 0.0};                                         // for gyro drift correction

    bool stable() const;
    bool upright() const;

    void resetState();
    void updateStateIMU(uint32_t currentTime, const Vector3<float>& accel, const Vector3<float>& gyro);
    void updateStatePT(uint32_t currentTime);
    void updateStateMag(const Vector3<float>& data);

    // const float* q; //quaternion storage for logging
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    struct __attribute__((packed)) Parameters {
        Parameters();
        bool verify() const {
            return true;
        }
        // state estimation parameters for tuning
        float state_estimation[2];  // Madwick 2Kp, 2Ki, Mahony Beta

        // limits for enabling motors
        float enable[2];  // variance and gravity angle
    } parameters;

    static_assert(sizeof(Parameters) == 2 * 2 * 4, "Data is not packed");

   private:
    static float fast_cosine(float x_deg);

    float mixRadians(float w1, float a1, float a2);
    uint32_t lastUpdateMicros = 0;  // 1.2 hrs should be enough

    Localization localization;
    Systems* sys_;
};  // end of class State

#define DEG2RAD 0.01745329251f

#endif
