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
#include "i2cManager.h"
#include "led.h"
#include "power.h"
#include "serial.h"
#include "state.h"
#include "version.h"
#include "devicename.h"
#include "config.h"
#include "stateFlag.h"
#include "kinematics.h"
#include "controlVectors.h"
#include "commandVector.h"

struct Systems {
    Systems();
    // subsystem objects initialize pins when created
    // Storing inside a struct forces the right initialization order
    StateFlag flag;

    Kinematics kinematics;
    ControlVectors control_vectors;
    CommandVector command_vector;

    I2CManager i2c;
    State state;
    LED led;

    BMP280 bmp;
    Imu imu;
    PowerMonitor pwr;
    PilotCommand pilot;
    Control control;
    SerialComm conf;

    ConfigID id;

    DeviceName name;

    Version version;
    PcbTransform pcb_transform;

    void parseConfig();
};

#endif /* end of include guard: SYSTEMS_H */
