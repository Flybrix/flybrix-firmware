/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <systems.h/cpp>

    Set of systems that form the controller.
*/
#ifndef SYSTEMS_H
#define SYSTEMS_H

#include "AK8963.h"
#include "BMP280.h"
#include "MPU9250.h"
#include "R415X.h"
#include "airframe.h"
#include "command.h"
#include "config.h"
#include "control.h"
#include "i2cManager.h"
#include "led.h"
#include "motors.h"
#include "power.h"
#include "serial.h"
#include "state.h"
#include "version.h"

struct Systems {
    Systems();
    // subsystem objects initialize pins when created
    // Storing inside a struct forces the right initialization order
    R415X receiver;

    I2CManager i2c;
    State state;
    LED led;

    BMP280 bmp;
    MPU9250 mpu;
    AK8963 mag;
    PowerMonitor pwr;
    Motors motors;
    Airframe airframe;
    PilotCommand pilot;
    Control control;
    SerialComm conf;

    ConfigID id;
};

#endif /* end of include guard: SYSTEMS_H */
