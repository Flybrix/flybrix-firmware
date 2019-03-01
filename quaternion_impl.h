/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef QUATERNION_IMPL_H
#define QUATERNION_IMPL_H

#include "quaternion.h"

template<typename Number>
Quaternion<Number>::Quaternion() : Quaternion(1, 0, 0, 0) {
}

template<typename Number>
Quaternion<Number>::Quaternion(Number w, Number x, Number y, Number z) : w{w}, x{x}, y{y}, z{z} {
}

template<typename Number>
bool Quaternion<Number>::isZero() const {
    return w == 0 && x == 0 && y == 0 && z == 0;
}

template<typename Number>
Number Quaternion<Number>::lengthSq() const {
    return w * w + x * x + y * y + z * z;
}

template<typename Number>
Quaternion<Number> Quaternion<Number>::conj() const {
    return {w, -x, -y, -z};
}

template<typename Number>
Quaternion<Number>& Quaternion<Number>::operator*=(Number v) {
    w *= v;
    x *= v;
    y *= v;
    z *= v;
    return *this;
}

template<typename Number>
Quaternion<Number>& Quaternion<Number>::operator+=(const Quaternion<Number>& q) {
    w += q.w;
    x += q.x;
    y += q.y;
    z += q.z;
    return *this;
}

template<typename Number>
Quaternion<Number>& Quaternion<Number>::operator-=(const Quaternion<Number>& q) {
    w -= q.w;
    x -= q.x;
    y -= q.y;
    z -= q.z;
    return *this;
}

template<typename Number>
RotationMatrix<Number> Quaternion<Number>::toRotation() const {
    RotationMatrix<Number> m;
    m(0, 0) = 1 - 2 * (y * y + z * z);
    m(0, 1) = 2 * (x * y - w * z);
    m(0, 2) = 2 * (w * y + x * z);

    m(1, 0) = 2 * (w * z + x * y);
    m(1, 1) = 1 - 2 * (x * x + z * z);
    m(1, 2) = 2 * (y * z - w * x);

    m(2, 0) = 2 * (x * z - w * y);
    m(2, 1) = 2 * (w * x + y * z);
    m(2, 2) = 1 - 2 * (x * x + y * y);
    return m;
}

template<typename Number>
Quaternion<Number> operator*(const Quaternion<Number>& p, const Quaternion<Number>& q) {
    return {
            p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
            p.x * q.w + p.w * q.x - p.z * q.y + p.y * q.z,
            p.y * q.w + p.z * q.x + p.w * q.y - p.x * q.z,
            p.z * q.w - p.y * q.x + p.x * q.y + p.w * q.z
    };
}

template<typename Number>
Quaternion<Number> operator*(const Quaternion<Number>& p, Number v) {
    return {
            p.w * v,
            p.x * v,
            p.y * v,
            p.z * v
    };
}

template<typename Number>
Quaternion<Number> operator+(const Quaternion<Number>& p, const Quaternion<Number>& q) {
    return {
            p.w + q.w,
            p.x + q.x,
            p.y + q.y,
            p.z + q.z
    };
}

template<typename Number>
Quaternion<Number> operator-(const Quaternion<Number>& p, const Quaternion<Number>& q) {
    return {
            p.w - q.w,
            p.x - q.x,
            p.y - q.y,
            p.z - q.z
    };
}

template<typename Number>
Quaternion<Number> operator*(const Quaternion<Number>& p, const Vector3<Number>& q) {
    return {
            -p.x * q.x - p.y * q.y - p.z * q.z,
            +p.w * q.x - p.z * q.y + p.y * q.z,
            +p.z * q.x + p.w * q.y - p.x * q.z,
            -p.y * q.x + p.x * q.y + p.w * q.z
    };
}

template<typename Number>
Number Quaternion<Number>::pitch() const {
    return std::atan2(w * x + y * z, 0.5 - x * x - y * y);
}

template<typename Number>
Number Quaternion<Number>::roll() const {
    return std::asin(2 * (w * y - x * z));
}

template<typename Number>
Number Quaternion<Number>::yaw() const {
    return std::atan2(w * z + x * y, 0.5 - y * y - z * z);
}

#endif  // QUATERNION_IMPL_H
