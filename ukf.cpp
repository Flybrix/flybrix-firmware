#include "ukf.h"

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

template <typename T, size_t N>
inline void subStates(const merwe::State<T, N>& in, merwe::State<T, N>& out) {
    for (size_t i = 0; i < N; ++i) {
        out[i] -= in[i];
    }
}

template <typename T, size_t N>
inline void addSelfProduct(const merwe::State<T, N>& in, T scale, merwe::Covariance<T, N>& out) {
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            out[i][j] += scale * in[i] * in[j];
        }
    }
}

}  // namespace

UKF::UKF() : x_{{0, 0, 0, 0, 0}}, P_{{{{0, 0, 0, 0, 0}}, {{0, 0, 0, 0, 0}}, {{0, 0, 0, 0, 0}}, {{0, 0, 0, 0, 0}}, {{0, 0, 0, 0, 0}}}}, weights_(5, ALPHA, BETA, KAPPA) {
    P_[StateFields::V_X][StateFields::V_X] = 100;
    P_[StateFields::V_Y][StateFields::V_Y] = 100;
    P_[StateFields::V_Z][StateFields::V_Z] = 100;
    P_[StateFields::P_Z][StateFields::P_Z] = 10000;
    P_[StateFields::H_GROUND][StateFields::H_GROUND] = 10000;
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
    for (size_t i = 0; i < 11; ++i) {
        subStates(x_, sigmas_f_[i]);
    }
    P_ = Q;
    addSelfProduct(sigmas_f_[0], weights_.covariance_center, P_);
    for (size_t i = 1; i < 11; ++i) {
        addSelfProduct(sigmas_f_[i], weights_.covariance_offset, P_);
    }
}

void update(float vx, float vy, float d_tof, float h_bar, float roll, float pitch, const merwe::Covariance<float, 4>& R) {
    // TODO
}
