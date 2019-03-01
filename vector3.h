/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef VECTOR3_H
#define VECTOR3_H

template<typename Number>
struct __attribute__((packed)) Vector3 final {
    Vector3();
    Vector3(Number x, Number y, Number z);

    template<typename OtherNumber>
    Vector3(const Vector3<OtherNumber>& v) : Vector3(v.x, v.y, v.z) {}

    Number x;
    Number y;
    Number z;

    Vector3<Number>& operator*=(Number scale);
    Vector3<Number> operator*(Number scale) const;
    Vector3<Number> operator/(Number scale) const;
    Vector3<Number> operator*(const Vector3<Number>& v) const;
    Vector3<Number>& operator+=(const Vector3<Number>& op);
    Vector3<Number>& operator-=(const Vector3<Number>& op);
    Vector3<Number> operator+(const Vector3<Number>& op) const;
    Vector3<Number> operator+(Number v) const;
    Vector3<Number> operator-(const Vector3<Number>& op) const;
    Vector3<Number> operator-(Number v) const;
    bool isZero() const;
    Number lengthSq() const;
    Vector3<Number> squared() const;
    Vector3<Number> projectOntoPlane(const Vector3<Number>& normal) const;
    Vector3<Number> projectOntoOrd(const Vector3<Number>& v) const;
};

template<typename Number>
Number dot(const Vector3<Number>& a, const Vector3<Number>& b);

template<typename Number>
Vector3<Number> cross(const Vector3<Number>& a, const Vector3<Number>& b);

static_assert(sizeof(Vector3<float>) == 12, "Vector3 data is not packed");

#include "vector3_impl.h"

#endif  // VECTOR3_H
