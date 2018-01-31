/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "debug.h"

#include "serial.h"

SerialComm* debug_serial_comm{nullptr};

void DebugSendString(const String& string) {
    if (!debug_serial_comm)
        return;
    debug_serial_comm->SendDebugString(string);
}
