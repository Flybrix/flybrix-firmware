#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <Arduino.h>
#include "cobs.h"

class SerialComm;

class Autopilot final {
   public:
    explicit Autopilot(SerialComm& serial);
    void start(uint32_t now);
    void run(uint32_t now);
    void stop();

   private:
    void readCobs();
    void handleCobs();

    bool running_{false};
    uint32_t start_time_{0};
    uint32_t wait_until_{0};
    SerialComm& serial_;
    CobsReaderBuffer data_input;
};

#endif  // AUTOPILOT_H
