#ifndef VECTOR3_H
#define VECTOR3_H

template <typename Number>
struct __attribute__((packed)) Vector3 {
    Vector3() : Vector3(0, 0, 0) {
    }

    Vector3(Number x, Number y, Number z) : x{x}, y{y}, z{z} {
    }

    template <typename OtherNumber>
    Vector3(const Vector3<OtherNumber>& v) : Vector3(v.x, v.y, v.z) {
    }

    Number x;
    Number y;
    Number z;

    Vector3<Number>& operator*=(Number scale) {
        x *= scale;
        y *= scale;
        z *= scale;
        return *this;
    }

    Vector3<Number> operator*(Number scale) const {
        return {x * scale, y * scale, z * scale};
    }

    Vector3<Number> operator/(Number scale) const {
        return {x / scale, y / scale, z / scale};
    }

    Vector3<Number> operator*(const Vector3<Number>& v) const {
        return {x * v.x, y * v.y, z * v.z};
    }

    Vector3<Number>& operator+=(const Vector3<Number>& op) {
        x += op.x;
        y += op.y;
        z += op.z;
        return *this;
    }

    Vector3<Number>& operator-=(const Vector3<Number>& op) {
        x -= op.x;
        y -= op.y;
        z -= op.z;
        return *this;
    }

    Vector3<Number> operator+(const Vector3<Number>& op) const {
        return {x + op.x, y + op.y, z + op.z};
    }

    Vector3<Number> operator+(Number v) const {
        return {x + v, y + v, z + v};
    }

    Vector3<Number> operator-(const Vector3<Number>& op) const {
        return {x - op.x, y - op.y, z - op.z};
    }

    Vector3<Number> operator-(Number v) const {
        return {x - v, y - v, z - v};
    }

    bool isZero() const {
        return x == 0 && y == 0 && z == 0;
    }

    Number lengthSq() const {
        return dot(*this, *this);
    }

    Vector3<Number> squared() const {
        return *this * *this;
    }
};

template <typename Number>
Number dot(const Vector3<Number>& a, const Vector3<Number>& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static_assert(sizeof(Vector3<float>) == 12, "Vector3 data is not packed");

#endif  // VECTOR3_H
