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

struct Systems;

class State {
   public:
    explicit State(Systems* sys);

    // timing
    uint32_t loopCount = 0;

    // Power
    uint16_t V0_raw = 0, I0_raw = 0, I1_raw = 0;  // raw ADC levels

    // MPU9250 and AK8963
    float R[3][3];                                                                  // rotation matrix from pcb to flyer frame
    float accel[3] = {0.0, 0.0, 0.0};                                               // g's        -- (x,y,z)
    float gyro[3] = {0.0, 0.0, 0.0};                                                // deg/sec    -- (x,y,z)
    float mag[3] = {0.0, 0.0, 0.0};                                                 // milligauss -- (x,y,z)
    float accel_filter[3] = {0.0, 0.0, 0.0}, accel_filter_sq[3] = {0.0, 0.0, 0.0};  // for stability variance calculation
    float gyro_filter[3] = {0.0, 0.0, 0.0};                                         // for gyro drift correction

    // Motors
    void processMotorEnablingIteration();  // must be called ~80 times to enable motors.
    void disableMotors();
    uint16_t enableAttempts = 0;  // increment when we're in the STATUS_ENABLING state

    void resetState();
    void updateStateIMU(uint32_t currentTime);
    void updateStatePT(uint32_t currentTime);
    void updateStateMag();

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
    bool stable();
    bool upright();
    float fast_cosine(float x_deg);

    float mixRadians(float w1, float a1, float a2);
    uint32_t lastUpdateMicros = 0;  // 1.2 hrs should be enough

    Localization localization;
    Systems* sys_;
};  // end of class State

#define DEG2RAD 0.01745329251f

#endif
