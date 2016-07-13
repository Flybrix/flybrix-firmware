/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <command.h/cpp>

    This code interprets the raw RC data into pilot intentions (stick inputs, mode changes, etc.)

*/

#ifndef command_h
#define command_h

#include "Arduino.h"
#include "R415X.h"

class State;

class PilotCommand {
   public:
    PilotCommand(State* state, R415X* receiver);
    void processCommands(void);

   private:
    State* state;
    R415X* receiver;

    boolean blockEnabling = true;
    boolean recentlyEnabled = false;
    uint16_t throttleHoldOff = 0;  // hold controls low for some time after enabling

};

#endif
