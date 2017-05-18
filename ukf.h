#ifndef UKF_H
#define UKF_H

#include "utility/merwePoints.h"

class UKF final {
   public:
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

    void predict(float dt, const merwe::Covariance<float, 5>& Q);

    void update(float vx, float vy, float d_tof, float h_bar, float roll, float pitch, const merwe::Covariance<float, 4>& R);

   private:
    enum StateFields {
        V_X = 0,
        V_Y = 1,
        P_Z = 2,
        V_Z = 3,
        H_GROUND = 4,
    };
    using SigmasF = std::array<merwe::State<float, 5>, 11>;
    using SigmasH = std::array<merwe::State<float, 4>, 11>;
    merwe::State<float, 5> x_;
    merwe::Covariance<float, 5> P_;
    SigmasF sigmas_f_;
    SigmasH sigmas_h_;
    merwe::Weights<float> weights_;
};

#endif  // UKF_H
