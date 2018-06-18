/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef debug_h
#define debug_h

#ifdef TESTSUITE

template <class... T>
void DebugPrint(T... args) {
}

template <class... T>
void DebugPrintf(T... args) {
}

#else

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

void DebugIndicatorRedOn();
void DebugIndicatorGreenOn();
void DebugIndicatorRedOff();
void DebugIndicatorGreenOff();

void DebugSetIndicatorOverride(bool mode);
bool DebugGetIndicatorOverride();

#endif

#endif
