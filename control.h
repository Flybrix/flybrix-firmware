/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <control.h/cpp>

    Takes pilot commands and state data and generates instantaneous control vectors that are passed over to airframe.

*/

#ifndef control_h
#define control_h

#include "Arduino.h"
#include "cascadedPID.h"

class PID;
class CONFIG_struct;
class State;

class Control {
   public:
    Control(State *state, CONFIG_struct& config);

    void parseConfig(CONFIG_struct& config);

    void calculateControlVectors();

    State *state;
    uint32_t lastUpdateMicros = 0;  // 1.2 hrs should be enough
    
    // unpack config.pidBypass for convenience
    bool pidEnabled[8]{false, false, false, false, false, false, false, false};

    // controllers
    CascadedPID thrust_pid, pitch_pid, roll_pid, yaw_pid;
};

#endif
