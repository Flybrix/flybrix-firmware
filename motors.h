/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <motors.h/cpp>

    Applies motor levels using output PWM timers

*/

#ifndef motors_h
#define motors_h

#include "Arduino.h"

class Motors {
   public:
    Motors();
    void updateAllChannels(bool enabled);

    uint16_t get(size_t index) {
        return output_[index];
    }

    void set(size_t index, uint16_t value) {
        output_[index] = value;
    }

    void reset() {
        for (auto& v : output_) {
            v = 0;
        }
    };

    template <typename Tstream>
    void writeTo(Tstream& output) {
        output.Append(output_);
    }

   private:
    uint16_t output_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

#endif
