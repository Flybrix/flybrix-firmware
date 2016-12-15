/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <debug.h/cpp>

    Utility functions for debugging
*/

#ifndef debug_h
#define debug_h

#include "Arduino.h"

class SerialComm;
extern SerialComm* debug_serial_comm;

void DebugSendString(const String& string);

template <class... T>
void DebugPrint(T... args) {
    DebugSendString(String(args...));
}

template <class... T>
void DebugPrintf(T... args) {
    if (!debug_serial_comm)
        return;
    char print_buffer[500];
    sprintf(print_buffer, args...);
    DebugPrint(print_buffer);
}

#endif
