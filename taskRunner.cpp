/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "taskRunner.h"

TaskRunner::TaskRunner(TaskPtr task, uint32_t desired_interval_us) {
    TaskRunner(task, desired_interval_us, true);
}

TaskRunner::TaskRunner(TaskPtr task, uint32_t desired_interval_us, bool running) : task{task}, desired_interval_us{desired_interval_us}, last_update_us{micros()}, running{running} {
}

bool TaskRunner::process() {
    uint32_t test{micros()};

    uint32_t delay{test - last_update_us};

    if (desired_interval_us > delay) {
        return false;
    }

    last_update_us = micros();
    bool did_something = task();
    if (did_something){
        work_count++;
    }
    logExecution(delay, micros() - last_update_us);
    return did_something;
}
