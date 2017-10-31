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
#include "kinematics.h"
#include "localization.h"
#include "utility/vector3.h"

struct Systems;

class State {
   public:
    explicit State(Systems* sys);

    // timing
    uint32_t loopCount = 0;

    void resetState();
    void updateLocalization(uint32_t currentTime, const Vector3<float>& accel, const Vector3<float>& rate_scaled);
    void readStatePT();
    void updateStateMag(const Vector3<float>& data);
    void updateFilter(uint32_t time);

    Vector3<float> getVelocity() {
        return localization.getVelocity();
    }

    const Kinematics& getKinematics() const {
        return kinematics;
    }

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
    Kinematics kinematics;
    Localization localization;
    Systems* sys_;
};  // end of class State

#define DEG2RAD 0.01745329251f

#endif
