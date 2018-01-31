/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef state_h
#define state_h

#include <cstdint>
#include "kinematics.h"
#include "localization.h"
#include "utility/vector3.h"

class State {
   public:
    State();

    // timing
    uint32_t loopCount = 0;

    void resetState();
    void updateLocalization(uint32_t currentTime, const Vector3<float>& accel, const Vector3<float>& rate_scaled);
    void readStatePT(uint32_t p0, uint32_t pressure, uint16_t temperature);
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
};  // end of class State

#define DEG2RAD 0.01745329251f

#endif
