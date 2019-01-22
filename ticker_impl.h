/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef TICKER_IMPL_H
#define TICKER_IMPL_H

#include "ticker.h"

template<typename Number>
bool Ticker<Number>::tick() {
    if (count_ == 0) {
        return false;
    }
    --count_;
    return true;
}

template<typename Number>
void Ticker<Number>::reset(Number ticks) {
    count_ = ticks;
}

#endif // TICKER_IMPL_H
