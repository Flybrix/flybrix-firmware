/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <debug.h/cpp>

    Utility functions for debugging
*/

#include "debug.h"

#include "serial.h"

SerialComm* debug_serial_comm{nullptr};

void DebugSendString(const String& string) {
    if (!debug_serial_comm)
        return;
    debug_serial_comm->SendDebugString(string);
}
