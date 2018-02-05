/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "taskRunner.h"
#include "loop_stopper.h"
#include "debug.h"

TaskRunner::TaskRunner(TaskPtr task, uint32_t desired_interval_us) : task{task}, desired_interval_us{desired_interval_us}, last_update_us{micros()}, enabled{true}, always_log_stats{false} {
}

TaskRunner::TaskRunner(TaskPtr task, uint32_t desired_interval_us, bool enabled) : task{task}, desired_interval_us{desired_interval_us}, last_update_us{micros()}, enabled{enabled}, always_log_stats{false} {
}

TaskRunner::TaskRunner(TaskPtr task, uint32_t desired_interval_us, bool enabled, bool always_log_stats) : task{task}, desired_interval_us{desired_interval_us}, last_update_us{micros()}, enabled{enabled}, always_log_stats{always_log_stats} {
}

bool TaskRunner::process() {
    uint32_t delay = micros() - last_update_us;
    
    if (desired_interval_us > delay) {
        return false;
    }

    last_update_us = micros();
    bool did_something = task();
    if (did_something){
        work_count++;
    }
    if (always_log_stats || !loops::used()) {       
        logExecution(delay, micros() - last_update_us); //increments log_count
    }
    return did_something;
}

