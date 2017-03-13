/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <command.h/cpp>

    This code interprets the raw RC data into pilot intentions (stick inputs, mode changes, etc.)

*/

#ifndef command_h
#define command_h

#include <cstdint>

struct CommandVector;
class State;
class StateFlag;
class R415X;

class PilotCommand {
   public:
    PilotCommand(State& state, R415X& receiver, StateFlag& flag, CommandVector& command_vector);
    void processCommands();

   private:
    State& state_;
    R415X& receiver_;
    StateFlag& flag_;
    CommandVector& command_vector_;

    bool blockEnabling{true};
    bool recentlyEnabled{false};
    uint16_t throttleHoldOff{0};  // hold controls low for some time after enabling
    uint8_t bluetoothTolerance{0};
    int16_t invalid_count{0};
};

#endif
