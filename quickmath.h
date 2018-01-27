/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef QUICKMATH_H
#define QUICKMATH_H

namespace quick {

float invSqrt(float);

float cos(float);
float sin(float);

template <typename T>
inline void normalize(T& v) {
    v *= invSqrt(v.lengthSq());
}

}  // namespace quick

#endif  // QUICKMATH_H
