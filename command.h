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

class PPMchannel {
   public:
    PPMchannel(){};

    uint16_t val = 1500;
    uint16_t mid = 1500;  // we allow the midpoint to be overwritten using trim
    static const uint16_t untrimmed_mid = 1500;
    static const uint16_t min = 1100;
    static const uint16_t max = 1900;

    void update(uint16_t newVal) {
        val = newVal;
    };

    boolean isExtraLow() {
        return ((val - min) < (max - min) / 5);
    };
    boolean isLow() {
        return ((val - min) < (max - min) / 10);
    };
    boolean isHigh() {
        return ((max - val) < (max - min) / 10);
    };
    boolean isMid() {
        return (abs(val - mid) < (max - min) / 10);
    };

    // R/C controllers are sold with a specified "mode" that is either "mode 1" or "mode 2"
    // our default settings assume a "mode 2" controller, but the channels can be remapped if desired.
    // "mode 2" puts "throttle/yaw" on left stick and "pitch/roll" on the right
    // "mode 1" puts "pitch/yaw" on left stick and "throttle/roll" on the right
    // in both modes, sticks "down" or "right" produce LOW ppm values, while sticks "up" or "left" give high values
    //
    // We want our sticks to map cleanly over to our flyer's coordinate system:
    //   ( +x = right, +y = forward, +z = up) which implies (+pitch = nose up, +roll = RHS down, +yaw = CCW viewed from above)
    //
    // ***** To match this convention, we must invert the direction of the pitch and roll commands *****
    //
    // This is done using 'CONFIG.data.commandInversion' at the end of command.cpp, when we scale for use in the state command variables.
    //
};

class PilotCommand {
   public:
    PilotCommand(State* state);
    void processCommands(void);

   private:
    State* state;
    void loadRxData();

    boolean blockEnabling = true;
    boolean recentlyEnabled = false;
    uint16_t throttleHoldOff = 0;  // hold controls low for some time after enabling

    PPMchannel throttle;
    PPMchannel pitch;
    PPMchannel roll;
    PPMchannel yaw;
    PPMchannel AUX1;
    PPMchannel AUX2;

    int16_t* throttle_command;
    int16_t* pitch_command;
    int16_t* roll_command;
    int16_t* yaw_command;
};

#endif
