#include "devicename.h"

#include "Arduino.h"

#include "debug.h"

DeviceName::DeviceName() : DeviceName("FLYBRIX") {
}

DeviceName::DeviceName(const String& name) {
    std::size_t l{name.length()};
    if (l > 8) {
        l = 8;
    }
    for (std::size_t i{0}; i < l; ++i) {
        value[i] = name.charAt(i);
    }
    for (std::size_t i{l}; i < MAX_NAME_LENGTH + 1; ++i) {
        value[i] = 0;
    }
}

bool badChar(char c);

bool DeviceName::verify() const {
    if (!value[0]) {
        DebugPrint("Name cannot be empty");
    }
    for (char c : value) {
        if (badChar(c)) {
            DebugPrint(
                "Illegal character in name!"
                " "
                "Names are limited to 0-9, a-z, A-Z, '_', '-'!");
            return false;
        }
        if (!c) {
            return true;
        }
    }
    DebugPrint("Given device name is too long (max 8 characters)!");
    return false;
}

bool badChar(char c) {
    if (!c) {
        return false;
    }
    if (c >= 'a' && c <= 'z') {
        return false;
    }
    if (c >= 'A' && c <= 'Z') {
        return false;
    }
    if (c >= '0' && c <= '9') {
        return false;
    }
    for (char c_legal : "_-") {
        if (c == c_legal) {
            return false;
        }
    }
    return true;
}
