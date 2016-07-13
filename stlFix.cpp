/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <stlFix.h/cpp>

    Implements missing elements for using certain STL libraries in Teensyduino.

*/

#include "stlFix.h"
#include "debug.h"

void std::__throw_bad_alloc() {
    DebugPrintf("SYSTEM ERROR: Unable to allocate memory");
    exit(1);
}

void std::__throw_length_error(const char* e) {
    DebugPrintf("SYSTEM ERROR: Length error: %s", e);
    exit(1);
}
