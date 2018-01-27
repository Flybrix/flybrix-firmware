/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com

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
