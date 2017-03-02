#ifndef CONTROL_VECTORS_H
#define CONTROL_VECTORS_H

struct ControlVectors final {
    float force_z{0.0};
    float torque_x{0.0};
    float torque_y{0.0};
    float torque_z{0.0};
};

static_assert(sizeof(ControlVectors) == 16, "Control vectors data is not packed");

#endif /* CONTROL_VECTORS_H */
