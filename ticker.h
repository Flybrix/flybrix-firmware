/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef TICKER_H
#define TICKER_H

template<typename Number>
class Ticker final {
public:
    bool tick();
    void reset(Number ticks);
private:
    Number count_{0};
};

#include "ticker_impl.h"

#endif // TICKER_H
