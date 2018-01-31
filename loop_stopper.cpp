/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "loop_stopper.h"

#include <Arduino.h>

namespace loops {
namespace {
static bool hadUnobservedStop{true};
static uint32_t starts_needed{1};
static uint32_t last_stop_micros{0};
static uint32_t overall_delay{0};
}  // namespace

bool consumeStop() {
    bool value{hadUnobservedStop};
    hadUnobservedStop = false;
    return value;
}

void stop() {
    hadUnobservedStop = true;
    if (starts_needed++) {
        return;
    }
    last_stop_micros = micros();
}

void start() {
    if (starts_needed-- > 1) {
        return;
    }
    overall_delay += micros() - last_stop_micros;
    starts_needed = 0;
}

bool stopped() {
    return starts_needed;
}

uint32_t delay() {
    return overall_delay;
}
}  // namespace loops
