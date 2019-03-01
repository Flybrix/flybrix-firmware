/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef UKF_H
#define UKF_H

#include <cmath>

#include "merwePoints.h"

class UKF final {
   public:
    struct Measurement {
        Measurement() : value{0.0f}, variance{1.0f}, weight{0.0f} {
        }

        Measurement(float value, float variance) : value{value}, variance{variance}, weight{1.0f} {};

       private:
        float value;
        float variance;
        // Unset measurements have zero weight
        float weight;

        friend UKF;
    };

    UKF();

    float vx() const {
        return x_[StateFields::V_X];
    }

    float vy() const {
        return x_[StateFields::V_Y];
    }

    float vz() const {
        return x_[StateFields::V_Z];
    }

    float elevation() const {
        return x_[StateFields::P_Z];
    }

    void setVelocityProcessNoiseVariance(float v) {
        Q_(StateFields::V_X, StateFields::V_X) = v;
        Q_(StateFields::V_Y, StateFields::V_Y) = v;
        Q_(StateFields::V_Z, StateFields::V_Z) = v;
    }

    void setElevationProcessNoiseVariance(float v) {
        Q_(StateFields::P_Z, StateFields::P_Z) = v;
    }

    void setElevationVelocityProcessNoiseCovariance(float v) {
        Q_(StateFields::V_Z, StateFields::P_Z) = v;
        Q_(StateFields::P_Z, StateFields::V_Z) = v;
    }

    void setGroundHeightProcessNoiseVariance(float v) {
        Q_(StateFields::H_GROUND, StateFields::H_GROUND) = v;
    }

    void predict(float dt);

    void update(Measurement vu, Measurement vv, Measurement d_tof, Measurement h_bar, float roll, float pitch);

   private:
    enum StateFields {
        V_X = 0,
        V_Y = 1,
        P_Z = 2,
        V_Z = 3,
        H_GROUND = 4,
    };

    enum SensorFields {
        V_U = 0,
        V_V = 1,
        D_TOF = 2,
        H_BAR = 3,
    };
    using SigmasF = std::array<merwe::State<float, 5>, 11>;
    merwe::Covariance<float, 5> Q_;
    merwe::State<float, 5> x_;
    merwe::Covariance<float, 5> P_;
    SigmasF sigmas_f_;
    merwe::Weights<float> weights_;
};

#endif  // UKF_H
