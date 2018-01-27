/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "stateFlag.h"

void StateFlag::set(FlagData bits) {
    flag_ |= bits;
}

void StateFlag::clear(FlagData bits) {
    flag_ &= ~bits;
}

void StateFlag::assign(FlagData bits, bool value) {
    if (value) {
        set(bits);
    } else {
        clear(bits);
    }
}

bool StateFlag::is(FlagData bits) const {
    return flag_ & bits;
}

FlagData StateFlag::value() const {
    return flag_;
}
