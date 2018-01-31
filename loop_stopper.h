/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef LOOP_STOPPER_H
#define LOOP_STOPPER_H

#include <cstdint>

namespace loops {
void stop();
void start();

class Stopper final {
   public:
    Stopper() {
        stop();
    }
    ~Stopper() {
        start();
    }
};

// Returns true if there was a stop since the last consume stop call
bool consumeStop();
bool stopped();
uint32_t delay();
}  // namespace loops

#endif  // LOOP_STOPPER_H
