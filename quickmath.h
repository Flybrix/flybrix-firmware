#ifndef QUICKMATH_H
#define QUICKMATH_H

namespace quick {
    
float invSqrt(float);

float fast_cosine(float);
float fast_sine(float);

template <typename T>
inline void normalize(T& v) {
    v *= invSqrt(v.lengthSq());
}

}

#endif  // QUICKMATH_H
