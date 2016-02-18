/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <motors.h/cpp>

    Applies motor levels using output PWM timers

*/

#ifndef motors_h
#define motors_h

#include "Arduino.h"

class State;

class Motors {
   public:
    Motors(State* state);
    void updateAllChannels();

   private:
    State* state;
};

// pin definitions
#define PWM7 5  // 64
#define PWM6 9  // 46
#define PWM5 21  // 63
#define PWM4 23  // 45
#define PWM3 20  // 62
#define PWM2 22  // 44
#define PWM1 32  // 41
#define PWM0 25  // 42

#endif
