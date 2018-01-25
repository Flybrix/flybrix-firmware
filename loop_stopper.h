#ifndef LOOP_STOPPER_H
#define LOOP_STOPPER_H

#include <cstdint>

namespace loops {
void stop();
void start();

class Stopper final {
   public:
    Stopper() {
        stop();
    }
    ~Stopper() {
        start();
    }
};

bool stopped();
uint32_t delay();
}  // namespace loops

#endif  // LOOP_STOPPER_H
