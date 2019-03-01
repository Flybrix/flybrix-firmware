#ifndef MERWE_POINTS_IMPL_H
#define MERWE_POINTS_IMPL_H

#include "merwePoints.h"

#include <cmath>

namespace merwe {

template<typename T, size_t N>
std::array<State<T, N>, 2 * N + 1> calcSigmaPoints(
        T alpha, T beta, T kappa, const State<T, N>& x, const Covariance<T, N>& p) {
    const Covariance<T, N> u(cholesky(p, alpha * alpha * (N + kappa)));
    std::array<State<T, N>, 2 * N + 1> sigmas;
    sigmas[0] = x;
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            sigmas[i + 1][j] = x[j] + u(i, j);
            sigmas[i + N + 1][j] = x[j] - u(i, j);
        }
    }
    return sigmas;
}

template<typename T>
Weights<T>::Weights(size_t N, T alpha, T beta, T kappa) {
    T lambda_plus_n{alpha * alpha * (N + kappa)};
    mean_offset = covariance_offset = 0.5 / lambda_plus_n;
    mean_center = 1 - N / lambda_plus_n;
    covariance_center = mean_center + 1 - alpha * alpha + beta;
}

}  // namespace merwe

#endif  // MERWE_POINTS_IMPL_H
