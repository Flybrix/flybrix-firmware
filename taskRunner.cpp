#include "taskRunner.h"

TaskRunner::TaskRunner(TaskPtr task, uint32_t desired_interval_us) : task{task}, desired_interval_us{desired_interval_us}, last_update_us{micros()} {
}

bool TaskRunner::process() {
    uint32_t now{micros()};

    uint32_t delay{now - last_update_us};

    if (desired_interval_us > delay) {
        return false;
    }

    last_update_us = now;
    if (!task()) {
        return false;
    }
    logExecution(delay - desired_interval_us, micros() - now);

    return true;
}
