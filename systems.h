/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <systems.h/cpp>

    Set of systems that form the controller.
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
#include "kinematics.h"
#include "controlVectors.h"
#include "commandVector.h"
#include "utility/rcHelpers.h"

struct Systems {
    Systems();
    // subsystem objects initialize pins when created
    // Storing inside a struct forces the right initialization order
    StateFlag flag;

    Kinematics kinematics;
    ControlVectors control_vectors;
    RcCommand command_vector;

    State state;
    LED led;

    RcMux<SerialRc, R415X> rc_mux;

    BMP280 bmp;
    Imu imu;
    PowerMonitor pwr;
    PilotCommand pilot;
    Control control;
    SerialComm conf;

    ConfigID id;

    DeviceName name;

    Version version;

    SerialRc& serialRc() {
        return rc_mux.source<0>();
    }

    R415X& radioReceiver() {
        return rc_mux.source<1>();
    }

    void parseConfig();
};

#endif /* end of include guard: SYSTEMS_H */
