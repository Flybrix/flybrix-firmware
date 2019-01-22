/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef SYSTEMS_H
#define SYSTEMS_H

#include "BMP280.h"
#include "imu.h"
#include "command.h"
#include "config.h"
#include "control.h"
#include "led.h"
#include "power.h"
#include "serial.h"
#include "state.h"
#include "version.h"
#include "devicename.h"
#include "stateFlag.h"
#include "controlVectors.h"
#include "autopilot.h"
#include "RcMux.h"

struct Systems {
    Systems();
    // subsystem objects initialize pins when created
    // Storing inside a struct forces the right initialization order
    StateFlag flag;

    ControlVectors control_vectors;
    RcCommand command_vector;

    State state;
    LED led;

    RcMux<SerialRc, Receiver> rc_mux;

    BMP280 bmp;
    Imu imu;
    PowerMonitor pwr;
    PilotCommand pilot;
    Control control;
    SerialComm conf;
    Autopilot autopilot;

    ConfigID id;

    DeviceName name;

    Version version;

    SerialRc& serialRc() {
        return rc_mux.source<0>();
    }

    Receiver& radioReceiver() {
        return rc_mux.source<1>();
    }

    void parseConfig();
};

#endif /* end of include guard: SYSTEMS_H */
