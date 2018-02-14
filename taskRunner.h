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
    return (uint32_t) (1000000.0f / hz);
}

struct StatTrack {
    void log(uint32_t value) {
        value_last = value;
        value_min = std::min(value_min, value);
        value_max = std::max(value_max, value);
        value_sum += value;
    }

    void reset() {
        value_last = 0;
        value_min = 0xFFFFFFFF;
        value_max = 0;
        value_sum = 0;
    }
    
    uint32_t value_last{0};
    uint32_t value_min{0xFFFFFFFF};
    uint32_t value_max{0};
    uint32_t value_sum{0};
};

class TaskRunner {
   public:
    TaskRunner(const char* _name, TaskPtr task, uint32_t desired_interval_us);
    TaskRunner(const char* _name, TaskPtr task, uint32_t desired_interval_us, bool enabled);
    TaskRunner(const char* _name, TaskPtr task, uint32_t desired_interval_us, bool enabled, bool always_log_stats);

    ~TaskRunner() {
        free(name);
    }
    
    void setDesiredInterval(uint32_t value_usec, uint32_t disable_threshold_usec) {
        desired_interval_us = value_usec;
        if (value_usec < disable_threshold_usec){
            if (!enabled)
                last_update_us = micros();
            enabled = true;
        }
        else {
            enabled = false;
        }
    }

    bool process(uint32_t expected_time_until_next_attempt_usec);
    bool isEnabled(){
        return enabled;
    }

    void reset(uint32_t reset_time_us) {
        last_update_us = reset_time_us;
    }

    void logExecution(uint32_t delay, uint32_t duration) {
        delay_track.log(delay);
        duration_track.log(duration);
        ++log_count;
    }

    void resetStats(){
        delay_track.reset();
        duration_track.reset();
        log_count = 0;
    }
    
    char * name;
    TaskPtr task;
    uint32_t desired_interval_us;  
    uint32_t last_update_us;
    StatTrack delay_track{0};
    StatTrack duration_track{0};
    uint32_t log_count{0};
    uint32_t work_count{0};
  
  private:
    bool enabled;
    bool always_log_stats;
};

#endif  // task_runner_h
