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

class SerialComm;

extern SerialComm* debug_serial_comm;

template<class... T>
void DebugPrint(T... args) {
    if (!debug_serial_comm)
        return;
    debug_serial_comm->SendDebugString(String(args...));
}

template<class... T>
void DebugPrintf(T... args) {
    if (!debug_serial_comm)
        return;
    char print_buffer[500];
    sprintf(print_buffer, args...);
    DebugPrint(print_buffer);
}

#ifdef DEBUG
#define assert(condition, message)                                                            \
    do {                                                                                      \
        if (!(condition)) {                                                                   \
            Serial.print("DEBUG: ");                                                          \
            Serial.print("Assertion `");                                                      \
            Serial.print(#condition);                                                         \
            Serial.print("` failed in ");                                                     \
            Serial.print((strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)); \
            Serial.print(" line ");                                                           \
            Serial.print(__LINE__);                                                           \
            Serial.print(": ");                                                               \
            Serial.println(message);                                                          \
        }                                                                                     \
    } while (false)
namespace debugdata {
void catchTime(size_t index);
}  // namespace debugdata

#define CATCH_TIME(index)            \
    do {                             \
        debugdata::catchTime(index); \
    } while (false)
float READ_TIME(size_t index);
#define DEBUGGING_ENABLED
#else
#define assert(condition, message) \
    do {                           \
    } while (false)
#define CATCH_TIME(index) \
    do {                  \
    } while (false)
float READ_TIME(size_t index);
#endif

#endif
