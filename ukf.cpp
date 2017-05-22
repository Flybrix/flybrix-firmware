#include "ukf.h"

#include <cmath>

namespace {
constexpr float ALPHA = 0.01;
constexpr float BETA = 2;
constexpr float KAPPA = -2;
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

    x_.setScaled(sigmas_f_[0], weights_.mean_center);
    for (size_t i = 1; i < 11; ++i) {
        x_.addScaled(sigmas_f_[i], weights_.mean_offset);
    }

    P_ = Q;
    Vector<float, 5> delta_f{sigmas_f_[0] - x_};
    P_.addCorrelation(delta_f, delta_f, weights_.covariance_center);
    for (size_t i = 1; i < 11; ++i) {
        Vector<float, 5> delta_f{sigmas_f_[i] - x_};
        P_.addCorrelation(delta_f, delta_f, weights_.covariance_offset);
    }
}

void UKF::update(float vu, float vv, float d_tof, float h_bar, float roll, float pitch, const merwe::Covariance<float, 4>& R) {
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

    merwe::State<float, 4> z;
    z[SensorFields::V_U] = vu;
    z[SensorFields::V_V] = vu;
    z[SensorFields::D_TOF] = d_tof;
    z[SensorFields::H_BAR] = h_bar;

    merwe::State<float, 4> z_mean;

    z_mean.setScaled(sigmas_h_[0], weights_.mean_center);
    for (size_t i = 1; i < 11; ++i) {
        z_mean.addScaled(sigmas_h_[i], weights_.mean_offset);
    }

    merwe::Covariance<float, 4> P_z{R};
    Matrix<float, 5, 4> y_z_cov;

    const merwe::State<float, 4> delta_h{sigmas_h_[0] - z_mean};
    P_z.addCorrelation(delta_h, delta_h, weights_.covariance_center);
    y_z_cov.addCorrelation(sigmas_f_[0] - x_, delta_h, weights_.covariance_center);

    for (size_t i = 1; i < 11; ++i) {
        const merwe::State<float, 4> delta_h{sigmas_h_[i] - z_mean};
        P_z.addCorrelation(delta_h, delta_h, weights_.covariance_offset);
        y_z_cov.addCorrelation(sigmas_f_[i] - x_, delta_h, weights_.covariance_center);
    }
    Matrix<float, 5, 4> K{y_z_cov * invertRootable(P_z)};
    x_ += K * (z - z_mean);
    Matrix<float, 5, 4> K_P_z{K * P_z};
    P_ -= multMatrixAndTransposeMatrix(K_P_z, K);
}
