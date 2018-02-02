/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "loop_stopper.h"

#include <Arduino.h>

namespace loops {
namespace {
static bool last_stop_has_been_processed{true};
static uint32_t starts_needed{1};
static uint32_t last_start_micros{0};
static uint32_t last_stop_micros{0};
static uint32_t overall_delay{0};
}  // namespace

bool wereStopped() {
    bool value{last_stop_has_been_processed};
    last_stop_has_been_processed = true;
    return !value;
}

uint32_t lastStart() {
    return last_start_micros;
}

void stop() {
    last_stop_has_been_processed = false;
    if (starts_needed++) {
        return;
    }
    last_stop_micros = micros();
}

void start() {
    if (starts_needed-- > 1) {
        return;
    }
    last_start_micros = micros();
    overall_delay += last_start_micros - last_stop_micros;
    starts_needed = 0;
}

bool stopped() {
    return starts_needed;
}

uint32_t delay() {
    return overall_delay;
}
}  // namespace loops
