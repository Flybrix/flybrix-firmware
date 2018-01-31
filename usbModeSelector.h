/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef usb_mode_selector_h
#define usb_mode_selector_h

#include <cstdint>

namespace usb_mode {
enum Mode : uint8_t {
    NONE = 0,
    BLUETOOTH_MIRROR = 1,
    PERFORMANCE_REPORT = 2,
};

void set(Mode mode);
Mode get();
}  // namespace usb_mode

#endif  // usb_mode_selector_h
