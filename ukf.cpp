/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

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
    setVelocityProcessNoiseVariance(0.64);
    setElevationProcessNoiseVariance(1);
    setElevationVelocityProcessNoiseCovariance(0.5);
    setGroundHeightProcessNoiseVariance(0.25);
}

void UKF::predict(float dt) {
    sigmas_f_ = merwe::calcSigmaPoints(ALPHA, BETA, KAPPA, x_, P_);
    for (merwe::State<float, 5>& sigma : sigmas_f_) {
        sigma[StateFields::P_Z] += sigma[StateFields::V_Z] * dt;
    }

    x_.setScaled(sigmas_f_[0], weights_.mean_center);
    for (size_t i = 1; i < 11; ++i) {
        x_.addScaled(sigmas_f_[i], weights_.mean_offset);
    }

    P_ = Q_ * dt;
    Vector<float, 5> delta_f{sigmas_f_[0] - x_};
    P_.addCorrelation(delta_f, delta_f, weights_.covariance_center);
    for (size_t i = 1; i < 11; ++i) {
        Vector<float, 5> delta_f{sigmas_f_[i] - x_};
        P_.addCorrelation(delta_f, delta_f, weights_.covariance_offset);
    }
}

using SigmasH = std::array<merwe::State<float, 4>, 11>;

void UKF::update(Measurement vu, Measurement vv, Measurement d_tof, Measurement h_bar, float roll, float pitch) {
    float cr = cos(roll);
    float cp = cos(pitch);
    float sr = sin(roll);
    float sp = sin(pitch);
    float height_divider{cp * cr};

    SigmasH sigmas_h;

    for (size_t i = 0; i < 11; ++i) {
        float vx{sigmas_f_[i][StateFields::V_X]};
        float vy{sigmas_f_[i][StateFields::V_Y]};
        float vz{sigmas_f_[i][StateFields::V_Z]};
        float pz{sigmas_f_[i][StateFields::P_Z]};
        float h{sigmas_f_[i][StateFields::H_GROUND]};

        // Multiply by zero for unset measurements, resulting in zero Z_mean
        sigmas_h[i][SensorFields::V_U] = vu.weight * (cp * vx - sp * vz);
        sigmas_h[i][SensorFields::V_V] = vv.weight * (sp * sr * vx + cr * vy + cp * sr * vz);
        sigmas_h[i][SensorFields::D_TOF] = d_tof.weight * ((pz - h) / height_divider);
        sigmas_h[i][SensorFields::H_BAR] = h_bar.weight * (pz);
    }

    // Unset measurements have zero Z, so Z - Z_mean = 0
    merwe::State<float, 4> z;
    z[SensorFields::V_U] = vu.value;
    z[SensorFields::V_V] = vv.value;
    z[SensorFields::D_TOF] = d_tof.value;
    z[SensorFields::H_BAR] = h_bar.value;

    merwe::State<float, 4> z_mean;

    // Unset measurements have zero Z, so Z - Z_mean = 0
    z_mean.setScaled(sigmas_h[0], weights_.mean_center);
    for (size_t i = 1; i < 11; ++i) {
        z_mean.addScaled(sigmas_h[i], weights_.mean_offset);
    }

    // Unset measurements have variance one, so they don't affect the inverse
    merwe::Covariance<float, 4> P_z;
    P_z(SensorFields::V_U, SensorFields::V_U) = vu.variance;
    P_z(SensorFields::V_V, SensorFields::V_V) = vv.variance;
    P_z(SensorFields::D_TOF, SensorFields::D_TOF) = d_tof.variance;
    P_z(SensorFields::H_BAR, SensorFields::H_BAR) = h_bar.variance;
    Matrix<float, 5, 4> y_z_cov;

    const merwe::State<float, 4> delta_h{sigmas_h[0] - z_mean};
    P_z.addCorrelation(delta_h, delta_h, weights_.covariance_center);
    y_z_cov.addCorrelation(sigmas_f_[0] - x_, delta_h, weights_.covariance_center);

    for (size_t i = 1; i < 11; ++i) {
        const merwe::State<float, 4> delta_h{sigmas_h[i] - z_mean};
        P_z.addCorrelation(delta_h, delta_h, weights_.covariance_offset);
        y_z_cov.addCorrelation(sigmas_f_[i] - x_, delta_h, weights_.covariance_offset);
    }
    Matrix<float, 5, 4> K{y_z_cov * invertRootable(P_z)};
    x_ += K * (z - z_mean);
    P_ -= multMatrixAndTransposeMatrix(y_z_cov, K);
}
