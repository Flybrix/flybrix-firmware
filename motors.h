/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef motors_h
#define motors_h

#include "Arduino.h"

class Motors {
   public:
    Motors();

    void updateAllChannels(bool enabled);

    void set(size_t index, uint16_t value) {
        output_[index] = value;
    }

    void reset() {
        for (auto& v : output_) {
            v = 0;
        }
    };

    template<typename Tstream>
    void writeTo(Tstream& output) const {
        output.Append(output_);
    }

   private:
    uint16_t output_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

#endif
