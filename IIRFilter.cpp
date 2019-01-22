/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "IIRFilter.h"

IIRFilter::IIRFilter(float output, float time_constant) : out_{output}, tau_{time_constant} {}

float IIRFilter::update(float in, float dt) {
    out_ = (in * dt + out_ * tau_) / (dt + tau_);
    return out_;
}
