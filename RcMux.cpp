/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "RcMux.h"

bool RcFilter::accepts(uint8_t source) const {
    return sources_ & source;
}

void RcFilter::update(uint8_t sources) {
    sources_ = sources;
}
