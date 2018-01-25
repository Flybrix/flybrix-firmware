#include "loop_stopper.h"

#include <Arduino.h>

namespace loops {
namespace {
static bool loops_stopped{true};
static uint32_t last_stop_micros{0};
static uint32_t overall_delay{0};
}  // namespace

void stop() {
    if (loops_stopped) {
        return;
    }
    last_stop_micros = micros();
    loops_stopped = true;
}

void start() {
    if (!loops_stopped) {
        return;
    }
    overall_delay += micros() - last_stop_micros;
    loops_stopped = false;
}

bool stopped() {
    return loops_stopped;
}

uint32_t delay() {
    return overall_delay;
}
}  // namespace loops
