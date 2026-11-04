/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

// Guards for function-local static objects ("static Foo foo;" inside a function), per the ARM C++ ABI:
// the guard is 32 bits and bit 0 means "already constructed".
//
// Teensyduino 1.59's Teensy 3 core ships weak defaults that read and write the guard as 64 bits. The read picks up
// 4 unrelated bytes next to the guard, so a static can be wrongly reported as constructed and never built, and the
// write corrupts those 4 bytes. FastLED.addLeds() creates its LED controller this way: the controller was skipped,
// its first virtual call jumped through a null vtable into the interrupt vector table, and the board hung before the
// main loop (no LEDs). Strong definitions here replace the core's weak ones.

#include <stdint.h>

__extension__ typedef int flybrix_guard_t __attribute__((mode(__DI__)));  // same type the core declares

extern "C" int __cxa_guard_acquire(flybrix_guard_t* g) {
    return !(*reinterpret_cast<volatile uint32_t*>(g) & 1);  // 1 = not constructed yet: caller constructs
}

extern "C" void __cxa_guard_release(flybrix_guard_t* g) {
    *reinterpret_cast<volatile uint32_t*>(g) = 1;
}

extern "C" void __cxa_guard_abort(flybrix_guard_t*) {
}
