/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "usbModeSelector.h"

namespace usb_mode {
namespace {
Mode current_mode{PERFORMANCE_REPORT};
}

void set(Mode mode) {
    current_mode = mode;
}

Mode get() {
    return current_mode;
}
}  // namespace usb_mode
