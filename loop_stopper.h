/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef LOOP_STOPPER_H
#define LOOP_STOPPER_H

#include <cstdint>
#include "debug.h"

namespace loops {
void stop();
void start();

class Stopper final {
  public:
    Stopper(const char * _reason) {
        stop();
        reason = strdup(_reason);
        start_usec = micros();
        delay_to_report = 0;
    }
    Stopper(const char * _reason, uint32_t delay) {
        stop();
        reason = strdup(_reason);
        start_usec = micros();
        delay_to_report = delay;
    }
    ~Stopper() {
        start();
        DebugPrintf("Paused %d usec to %s.", (delay_to_report>0) ? delay_to_report : micros()-start_usec, reason);
        free(reason);
    }
  private:
    char * reason;
    uint32_t start_usec;
    uint32_t delay_to_report;
};

bool used();
void reset();
bool stopped();
uint32_t delay();
uint32_t lastStart();
}  // namespace loops

#endif  // LOOP_STOPPER_H
