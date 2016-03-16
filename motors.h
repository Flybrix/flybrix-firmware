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

#endif
