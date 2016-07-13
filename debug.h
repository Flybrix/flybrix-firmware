/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <debug.h/cpp>

    Utility functions for debugging
*/

#ifndef debug_h
#define debug_h

#include <string.h>

#include "serial.h"

extern SerialComm* debug_serial_comm;

template <class... T>
void DebugPrint(T... args) {
    if (!debug_serial_comm)
        return;
    debug_serial_comm->SendDebugString(String(args...));
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
