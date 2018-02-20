/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "loop_stopper.h"

#include <Arduino.h>
#include "led.h"

namespace loops {
namespace {
LED* led{nullptr};
static bool ready{true};
static uint32_t starts_needed{1};
static uint32_t last_start_micros{0};
static uint32_t last_stop_micros{0};
static uint32_t overall_delay{0};

inline void updateIndicator() {
    if (!led) {
        return;
    }
    led->forceGreenIndicator(stopped());
}
}  // namespace

void reset() {
    ready = true;
    updateIndicator();
}

bool used() {
    return !ready;
}

uint32_t lastStart() {
    return last_start_micros;
}

void stop() {
    ready = false;
    if (starts_needed++) {
        return;
    }
    last_stop_micros = micros();
    updateIndicator();
}

void start() {
    if (starts_needed-- > 1) {
        return;
    }
    last_start_micros = micros();
    overall_delay += last_start_micros - last_stop_micros;
    starts_needed = 0;
    updateIndicator();
}

bool stopped() {
    return starts_needed;
}

uint32_t delay() {
    return overall_delay;
}

void setLedIndicator(LED* value) {
    led = value;
}

}  // namespace loops
