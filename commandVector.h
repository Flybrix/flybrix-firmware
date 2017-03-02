#ifndef COMMAND_VECTOR_H
#define COMMAND_VECTOR_H

#include <cstdint>

struct CommandVector final {
    // bitfield order is {AUX1_low, AUX1_mid, AUX1_high, AUX2_low, AUX2_mid, AUX2_high, x, x} (LSB-->MSB)
    uint8_t aux_mask{0};
    float throttle{0.0};
    float pitch{0.0};
    float roll{0.0};
    float yaw{0.0};
};

#endif /* COMMAND_VECTOR_H */
