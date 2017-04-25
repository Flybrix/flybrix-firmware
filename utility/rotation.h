#ifndef ROTATION_H
#define ROTATION_H

#include "vector3.h"

template <typename Number>
struct RotationMatrix {
    RotationMatrix() : fields_{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} {
    }

    void applyTo(Number vector[3]) const {
        Number x{vector[0]};
        Number y{vector[1]};
        Number z{vector[2]};
        vector[0] = fields_[0][0] * x + fields_[0][1] * y + fields_[0][2] * z;
        vector[1] = fields_[1][0] * x + fields_[1][1] * y + fields_[1][2] * z;
        vector[2] = fields_[2][0] * x + fields_[2][1] * y + fields_[2][2] * z;
    }

    void applyTo(Vector3<Number>& vector) const {
        Number x{vector.x};
        Number y{vector.y};
        Number z{vector.z};
        vector.x = fields_[0][0] * x + fields_[0][1] * y + fields_[0][2] * z;
        vector.y = fields_[1][0] * x + fields_[1][1] * y + fields_[1][2] * z;
        vector.z = fields_[2][0] * x + fields_[2][1] * y + fields_[2][2] * z;
    }

    Number fields_[3][3];
};

#endif  // ROTATION_H
