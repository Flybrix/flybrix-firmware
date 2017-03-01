#include "stateFlag.h"

void StateFlag::set(FlagData bits) {
    flag_ |= bits;
}

void StateFlag::clear(FlagData bits) {
    flag_ &= ~bits;
}

bool StateFlag::is(FlagData bits) const {
    return flag_ & bits;
}

FlagData StateFlag::value() const {
    return flag_;
}
