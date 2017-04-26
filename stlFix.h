/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <stlFix.h/cpp>

    Implements missing elements for using certain STL libraries in Teensyduino.

*/

#ifndef STL_FIX_H
#define STL_FIX_H

namespace std {
void __throw_bad_function_call();
void __throw_bad_alloc();
void __throw_length_error(const char* e);
}

#endif /* end of include guard: STL_FIX_H */
