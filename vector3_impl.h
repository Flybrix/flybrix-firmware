/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef VECTOR3_IMPL_H
#define VECTOR3_IMPL_H

#include "vector3.h"

template<typename Number>
Vector3<Number>::Vector3() : Vector3(0, 0, 0) {
}

template<typename Number>
Vector3<Number>::Vector3(Number x, Number y, Number z) : x{x}, y{y}, z{z} {
}

template<typename Number>
Vector3<Number>& Vector3<Number>::operator*=(Number scale) {
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
}

template<typename Number>
Vector3<Number> Vector3<Number>::operator*(Number scale) const {
    return {x * scale, y * scale, z * scale};
}

template<typename Number>
Vector3<Number> Vector3<Number>::operator/(Number scale) const {
    return {x / scale, y / scale, z / scale};
}

template<typename Number>
Vector3<Number> Vector3<Number>::operator*(const Vector3<Number>& v) const {
    return {x * v.x, y * v.y, z * v.z};
}

template<typename Number>
Vector3<Number>& Vector3<Number>::operator+=(const Vector3<Number>& op) {
    x += op.x;
    y += op.y;
    z += op.z;
    return *this;
}

template<typename Number>
Vector3<Number>& Vector3<Number>::operator-=(const Vector3<Number>& op) {
    x -= op.x;
    y -= op.y;
    z -= op.z;
    return *this;
}

template<typename Number>
Vector3<Number> Vector3<Number>::operator+(const Vector3<Number>& op) const {
    return {x + op.x, y + op.y, z + op.z};
}

template<typename Number>
Vector3<Number> Vector3<Number>::operator+(Number v) const {
    return {x + v, y + v, z + v};
}

template<typename Number>
Vector3<Number> Vector3<Number>::operator-(const Vector3<Number>& op) const {
    return {x - op.x, y - op.y, z - op.z};
}

template<typename Number>
Vector3<Number> Vector3<Number>::operator-(Number v) const {
    return {x - v, y - v, z - v};
}

template<typename Number>
bool Vector3<Number>::isZero() const {
    return x == 0 && y == 0 && z == 0;
}

template<typename Number>
Number Vector3<Number>::lengthSq() const {
    return dot(*this, *this);
}

template<typename Number>
Vector3<Number> Vector3<Number>::squared() const {
    return *this * *this;
}

template<typename Number>
Vector3<Number> Vector3<Number>::projectOntoPlane(const Vector3<Number>& normal) const {
    return *this - projectOntoOrd(normal);
}

template<typename Number>
Vector3<Number> Vector3<Number>::projectOntoOrd(const Vector3<Number>& v) const {
    return v * dot(*this, v);
}

template<typename Number>
Number dot(const Vector3<Number>& a, const Vector3<Number>& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template<typename Number>
Vector3<Number> cross(const Vector3<Number>& a, const Vector3<Number>& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static_assert(sizeof(Vector3<float>) == 12, "Vector3 data is not packed");

#endif  // VECTOR3_IMPL_H
