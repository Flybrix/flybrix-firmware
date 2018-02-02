/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef task_runner_h
#define task_runner_h

#include <Arduino.h>

using TaskPtr = bool (*)();

constexpr uint32_t hzToMicros(float hz) {
    return 1000000 / hz;
}

struct StatTrack {
    void log(uint32_t value) {
        value_last = value;
        value_min = std::min(value_min, value);
        value_max = std::max(value_max, value);
        value_sum += value;
    }

    uint32_t value_last{0};
    uint32_t value_min{0xFFFFFFFF};
    uint32_t value_max{0};
    uint32_t value_sum{0};
};

class TaskRunner {
   public:
    TaskRunner(TaskPtr task, uint32_t desired_interval_us);
    TaskRunner(TaskPtr task, uint32_t desired_interval_us, bool running);

    void setDesiredInterval(uint32_t value) {
        desired_interval_us = value;
    }

    bool process();

    void reset() {
        last_update_us = micros();
    }

    void logExecution(uint32_t delay, uint32_t duration) {
        delay_track.log(delay);
        duration_track.log(duration);
        ++call_count;
    }

    TaskPtr task;
    uint32_t desired_interval_us;
    bool running{true};
    uint32_t last_update_us;
    StatTrack delay_track{0};
    StatTrack duration_track{0};
    uint32_t call_count{0};
    uint32_t work_count{0};
};

#endif  // task_runner_h
