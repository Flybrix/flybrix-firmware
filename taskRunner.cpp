#include "taskRunner.h"

TaskRunner::TaskRunner(TaskPtr task, uint32_t desired_interval_us) : task{task}, desired_interval_us{desired_interval_us}, last_update_us{micros()} {
}

bool TaskRunner::process() {
    uint32_t now{micros()};

    if (last_update_us + desired_interval_us > now) {
        return false;
    }

    last_update_us = now;
    task();
    logExecution(now - last_update_us - desired_interval_us, micros() - now);

    return true;
}
