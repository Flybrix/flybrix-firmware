/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "stlFix.h"
#include "debug.h"

void std::__throw_bad_function_call() {
    DebugPrintf("SYSTEM ERROR: Unable to call function");
    exit(1);
}

void std::__throw_bad_alloc() {
    DebugPrintf("SYSTEM ERROR: Unable to allocate memory");
    exit(1);
}

void std::__throw_length_error(const char* e) {
    DebugPrintf("SYSTEM ERROR: Length error: %s", e);
    exit(1);
}
