#ifndef COMMAND_VECTOR_H
#define COMMAND_VECTOR_H

#include <cstdint>

struct CommandVector final {
    enum class Source {
        None,
        Radio,
        Bluetooth,
    };

    enum class AUX : uint8_t {
        None = 0,
        Low = 1,
        Mid = 2,
        High = 4,
    };

    // bitfield order is {AUX1_low, AUX1_mid, AUX1_high, AUX2_low, AUX2_mid, AUX2_high, x, x} (LSB-->MSB)
    AUX aux1{AUX::None};
    AUX aux2{AUX::None};
    int16_t throttle{0};
    int16_t pitch{0};
    int16_t roll{0};
    int16_t yaw{0};
    Source source{Source::None};

    void clearBluetoothState() {
        if (source == Source::Bluetooth) {
            source = Source::None;
        }
    }

    void parseBools(bool l1, bool m1, bool h1, bool l2, bool m2, bool h2) {
        aux1 = l1 ? AUX::Low : m1 ? AUX::Mid : h1 ? AUX::High : AUX::None;
        aux2 = l2 ? AUX::Low : m2 ? AUX::Mid : h2 ? AUX::High : AUX::None;
    }

    void parseAuxMask(uint8_t auxmask) {
        parseBools(auxmask & 1, auxmask & 2, auxmask & 4, auxmask & 8, auxmask & 16, auxmask & 32);
    }

    uint8_t auxMask() const {
        return uint8_t(aux1) | (uint8_t(aux2) << 3);
    }
};

#endif /* COMMAND_VECTOR_H */
