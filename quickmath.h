#ifndef QUICKMATH_H
#define QUICKMATH_H

namespace quick {
float invSqrt(float);

template <typename T>
inline void normalize(T& v) {
    v *= invSqrt(v.lengthSq());
}
}

#endif  // QUICKMATH_H
