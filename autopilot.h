/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <Arduino.h>
#include "cobs.h"
#include "ClockTime.h"

class SerialComm;

class Autopilot final {
   public:
    explicit Autopilot(SerialComm& serial);
    void start(ClockTime now);
    bool run(ClockTime now);
    void stop();

   private:
    void readCobs();
    void handleCobs();

    bool running_{false};
    ClockTime start_time_{ClockTime::zero()};
    uint32_t wait_until_{0};
    SerialComm& serial_;
    CobsReaderBuffer data_input;
};

#endif  // AUTOPILOT_H
