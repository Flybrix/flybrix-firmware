/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "taskRunner.h"
#include "loop_stopper.h"
#include "debug.h"

TaskRunner::TaskRunner(const char* _name, TaskPtr task, uint32_t desired_interval_us) : task{task}, desired_interval_us{desired_interval_us}, last_update_us{micros()}, enabled{true}, always_log_stats{false} {
    name = strdup(_name);
}

TaskRunner::TaskRunner(const char* _name, TaskPtr task, uint32_t desired_interval_us, bool enabled) : task{task}, desired_interval_us{desired_interval_us}, last_update_us{micros()}, enabled{enabled}, always_log_stats{false} {
    name = strdup(_name);
}

TaskRunner::TaskRunner(const char* _name, TaskPtr task, uint32_t desired_interval_us, bool enabled, bool always_log_stats) : task{task}, desired_interval_us{desired_interval_us}, last_update_us{micros()}, enabled{enabled}, always_log_stats{always_log_stats} {
    name = strdup(_name);
}

bool TaskRunner::process(uint32_t expected_time_until_next_attempt_usec) {
    uint32_t delay = micros() - last_update_us;

    if ( (delay + expected_time_until_next_attempt_usec) < desired_interval_us ) {
        return false; // we can wait
    }

    last_update_us = micros();
    bool did_something = task();
    if (did_something){
        work_count++;
    }
    if ( always_log_stats || !loops::used() ) {       
        logExecution(delay, micros() - last_update_us); //increments log_count
    }
    return did_something;
}

