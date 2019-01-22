/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef RC_MUX_IMPL_H
#define RC_MUX_IMPL_H

#include "RcMux.h"

template<typename... Tsrcs>
RcState RcMux<Tsrcs...>::query() {
    RcState state(queryHelper());
    if (invalid_count_ <= 0) {
        invalid_count_ = 0;
        valid_ = true;
    }
    if (invalid_count_ >= 80) {
        invalid_count_ = 80;
        valid_ = false;
    }
    if (!valid_) {
        state.state = RcState::State::Timeout;
    }
    return state;
}

template<typename... Tsrcs>
void RcMux<Tsrcs...>::setFilter(uint8_t filter) {
    filter_.update(filter);
}

template<typename Tsrc>
const Tsrc& RcTracker<Tsrc>::source() const {
    return source_;
}

template<typename Tsrc>
Tsrc& RcTracker<Tsrc>::source() {
    return source_;
}

template<typename Tsrc>
RcState RcTracker<Tsrc>::query() {
    RcState state(source_.query());
    if (state.state == RcState::State::Ok) {
        tolerance_.reset(Tsrc::refresh_delay_tolerance);
        cache_ = state.command;
    } else {
        state.state = tolerance_.tick() ? RcState::State::Waiting : RcState::State::Timeout;
        state.command = cache_;
    }
    return state;
}

#endif // RC_MUX_IMPL_H
