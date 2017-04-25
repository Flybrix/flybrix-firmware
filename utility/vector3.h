#ifndef VECTOR3_H
#define VECTOR3_H

template <typename Number>
struct __attribute__((packed)) Vector3 {
    Number x;
    Number y;
    Number z;
};

static_assert(sizeof(Vector3<float>) == 12, "Vector3 data is not packed");

#endif  // VECTOR3_H
