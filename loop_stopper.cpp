#include "loop_stopper.h"

#include <Arduino.h>

namespace loops {
namespace {
static uint32_t starts_needed{1};
static uint32_t last_stop_micros{0};
static uint32_t overall_delay{0};
}  // namespace

void stop() {
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
