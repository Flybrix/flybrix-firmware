/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <debug.h/cpp>

    Utility functions for debugging
*/

#include "debug.h"
#include <Arduino.h>
#include <vector>

SerialComm* debug_serial_comm{nullptr};

namespace debugdata {

std::vector<float> delta_times;

unsigned long previous_timestamp = micros();

void catchTime(size_t index) {
    unsigned long timestamp = micros();
    while (index >= delta_times.size())
        delta_times.push_back(-1.0f);  // Use -1.0f as a flag that the variable is uninitialized
    if (delta_times[index] < 0.0f) {
        delta_times[index] = static_cast<float>(timestamp - previous_timestamp);
    } else {
        delta_times[index] = 0.99f * delta_times[index] + 0.01f * (timestamp - previous_timestamp);
        previous_timestamp = timestamp;
    }
}

}  // namespace debugdata

float READ_TIME(size_t index) {
    return index < debugdata::delta_times.size() ? debugdata::delta_times[index] : 0.0f;
}
