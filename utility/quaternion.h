#ifndef QUATERNION_H
#define QUATERNION_H

template <typename Number>
struct Quaternion {
    Quaternion() : Quaternion(1, 0, 0, 0) {
    }
    Quaternion(Number w, Number x, Number y, Number z) : w{w}, x{x}, y{y}, z{z} {
    }
    Number w;
    Number x;
    Number y;
    Number z;
};

#endif  // QUATERNION_H
