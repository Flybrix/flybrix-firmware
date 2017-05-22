#include "ukf.h"

#include <cmath>

namespace {
constexpr float ALPHA = 0.01;
constexpr float BETA = 2;
constexpr float KAPPA = -2;

template <typename T, size_t N>
inline void setScaled(const merwe::State<T, N>& in, T scale, merwe::State<T, N>& out) {
    for (size_t i = 0; i < N; ++i) {
        out[i] = in[i] * scale;
    }
}

template <typename T, size_t N>
inline void addScaled(const merwe::State<T, N>& in, T scale, merwe::State<T, N>& out) {
    for (size_t i = 0; i < N; ++i) {
        out[i] += in[i] * scale;
    }
}

}  // namespace

UKF::UKF() : weights_(5, ALPHA, BETA, KAPPA) {
    P_(StateFields::V_X, StateFields::V_X) = 100;
    P_(StateFields::V_Y, StateFields::V_Y) = 100;
    P_(StateFields::V_Z, StateFields::V_Z) = 100;
    P_(StateFields::P_Z, StateFields::P_Z) = 10000;
    P_(StateFields::H_GROUND, StateFields::H_GROUND) = 10000;
}

void UKF::predict(float dt, const merwe::Covariance<float, 5>& Q) {
    sigmas_f_ = merwe::calcSigmaPoints(ALPHA, BETA, KAPPA, x_, P_);
    for (merwe::State<float, 5>& sigma : sigmas_f_) {
        sigma[StateFields::P_Z] += sigma[StateFields::V_Z] * dt;
    }
    setScaled(sigmas_f_[0], weights_.mean_center, x_);
    for (size_t i = 1; i < 11; ++i) {
        addScaled(sigmas_f_[i], weights_.mean_offset, x_);
    }
    P_ = Q;
    Vector<float, 5> delta_f{sigmas_f_[0] - x_};
    P_.addCorrelation(delta_f, delta_f, weights_.covariance_center);
    for (size_t i = 1; i < 11; ++i) {
        Vector<float, 5> delta_f{sigmas_f_[i] - x_};
        P_.addCorrelation(delta_f, delta_f, weights_.covariance_offset);
    }
}

void UKF::update(float vx, float vy, float d_tof, float h_bar, float roll, float pitch, const merwe::Covariance<float, 4>& R) {
    float cr = cos(roll);
    float cp = cos(pitch);
    float sr = sin(roll);
    float sp = sin(pitch);
    float height_divider{cp * cr};
    for (size_t i = 0; i < 11; ++i) {
        float vx{sigmas_f_[i][StateFields::V_X]};
        float vy{sigmas_f_[i][StateFields::V_Y]};
        float vz{sigmas_f_[i][StateFields::V_Z]};
        float pz{sigmas_f_[i][StateFields::P_Z]};
        float h{sigmas_f_[i][StateFields::H_GROUND]};
        sigmas_h_[i][SensorFields::V_U] = cp * vx - sp * vz;
        sigmas_h_[i][SensorFields::V_V] = sp * sr * vx + cr * vy + cp * sr * vz;
        sigmas_h_[i][SensorFields::D_TOF] = (pz - h) / height_divider;
        sigmas_h_[i][SensorFields::H_BAR] = pz;
    }
    merwe::State<float, 4> z_mean;

    setScaled(sigmas_h_[0], weights_.mean_center, z_mean);
    for (size_t i = 1; i < 11; ++i) {
        addScaled(sigmas_h_[i], weights_.mean_offset, z_mean);
    }

    merwe::Covariance<float, 4> P_z{R};
    const merwe::State<float, 4> delta_h{sigmas_h_[0] - z_mean};
    P_z.addCorrelation(delta_h, delta_h, weights_.covariance_center);

    for (size_t i = 1; i < 11; ++i) {
        const merwe::State<float, 4> delta_h{sigmas_h_[i] - z_mean};
        P_z.addCorrelation(delta_h, delta_h, weights_.covariance_offset);
    }
    // TODO
    // Calculate P_z^-1
    // Calculate K
    // Calculate x & P
}
