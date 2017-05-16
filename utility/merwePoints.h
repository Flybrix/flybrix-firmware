#ifndef MERWE_POINTS_H
#define MERWE_POINTS_H

#include <array>

namespace merwe {
template <typename T, size_t N>
using State = std::array<T, N>;

template <typename T, size_t N>
using Covariance = std::array<std::array<T, N>, N>;

template <typename T, size_t N>
std::array<State<T, N>, 2 * N + 1> calcSigmaPoints(T alpha, T beta, T kappa, const State<T, N>& x, const Covariance<T, N>& p);

template <typename T>
struct Weights {
    T mean_center_;
    T mean_offset_;
    T covariance_center_;
    T covariance_offset_;
};

template <typename T>
Weights<T> calcWeights(size_t N, T alpha, T beta, T kappa);
}

#include "merwePoints_impl.h"

#endif  // MERWE_POINTS_H
