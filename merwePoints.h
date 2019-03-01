#ifndef MERWE_POINTS_H
#define MERWE_POINTS_H

#include <array>

#include "linalg.h"

namespace merwe {
template<typename T, size_t N>
using State = Vector<T, N>;

template<typename T, size_t N>
using Covariance = Matrix<T, N, N>;

template<typename T, size_t N>
std::array<State<T, N>, 2 * N + 1>
calcSigmaPoints(T alpha, T beta, T kappa, const State<T, N>& x, const Covariance<T, N>& p);

template<typename T>
struct Weights {
    Weights(size_t N, T alpha, T beta, T kappa);
    T mean_center;
    T mean_offset;
    T covariance_center;
    T covariance_offset;
};
}

#include "merwePoints_impl.h"

#endif  // MERWE_POINTS_H
