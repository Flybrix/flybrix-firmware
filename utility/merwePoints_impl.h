#ifndef MERWE_POINTS_IMPL_H
#define MERWE_POINTS_IMPL_H

#include "merwePoints.h"

#include <cmath>

namespace merwe {
namespace {

// https://en.wikipedia.org/wiki/Cholesky_decomposition#The_Cholesky.E2.80.93Banachiewicz_and_Cholesky.E2.80.93Crout_algorithms
template <typename T, size_t N>
Covariance<T, N> cholesky(const Covariance<T, N>& a, T scaling) {
    Covariance<T, N> l{};
    for (size_t i = 0; i < N; ++i) {
        T diag = a[i][i] * scaling;
        for (size_t j = 0; j < i; ++j) {
            T v{a[i][j] * scaling};
            for (size_t k = 0; k < j; ++k) {
                v -= l[i][k] * l[j][k];
            }
            v /= l[j][j];
            l[i][j] = v;
            diag -= v * v;
        }
        l[i][i] = sqrt(diag);
    }
    return l;
}

}  // namespace

template <typename T, size_t N>
std::array<State<T, N>, 2 * N + 1> calcSigmaPoints(T alpha, T beta, T kappa, const State<T, N>& x, const Covariance<T, N>& p) {
    T lambda_plus_n{alpha * alpha * (N + kappa)};
    const Covariance<T, N> u{cholesky(p, lambda_plus_n)};
    std::array<State<T, N>, 2 * N + 1> sigmas;
    sigmas[0] = x;
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            sigmas[i + 1][j] = x[j] + u[i][j];
            sigmas[i + N + 1][j] = x[j] - u[i][j];
        }
    }
    return sigmas;
}

template <typename T>
Weights<T> calcWeights(size_t N, T alpha, T beta, T kappa) {
    T lambda_plus_n{alpha * alpha * (N + kappa)};
    Weights<T> result;
    result.mean_offset_ = result.covariance_offset_ = 0.5 / lambda_plus_n;
    result.mean_center_ = 1 - N / lambda_plus_n;
    result.covariance_center_ = result.mean_center + 1 - alpha * alpha + beta;
    return result;
}

}  // namespace merwe

#endif  // MERWE_POINTS_IMPL_H
