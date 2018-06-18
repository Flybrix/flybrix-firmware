/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "debug.h"

#include "board.h"
#include "serial.h"

SerialComm* debug_serial_comm{nullptr};

void DebugSendString(const String& string) {
    if (!debug_serial_comm)
        return;
    debug_serial_comm->SendDebugString(string);
}

bool indicator_override{false};

void DebugSetIndicatorOverride(bool mode) {
    indicator_override = mode;
    DebugIndicatorRedOff();
    DebugIndicatorGreenOff();
}

bool DebugGetIndicatorOverride() {
    return indicator_override;
}

void DebugIndicatorRedOn() {
    if (indicator_override) {
        digitalWriteFast(board::RED_LED, HIGH);
    }
}

void DebugIndicatorGreenOn() {
    if (indicator_override) {
        digitalWriteFast(board::GREEN_LED, HIGH);
    }
}

void DebugIndicatorRedOff() {
    if (indicator_override) {
        digitalWriteFast(board::RED_LED, LOW);
    }
}

void DebugIndicatorGreenOff() {
    if (indicator_override) {
        digitalWriteFast(board::GREEN_LED, LOW);
    }
}
