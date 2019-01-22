/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef RC_MUX_H
#define RC_MUX_H

#include <cstdint>
#include <tuple>

#include "ticker.h"
#include "RcCommand.h"
#include "RcState.h"

class RcFilter final {
public:
    bool accepts(uint8_t source) const;
    void update(uint8_t sources);
private:
    uint8_t sources_{255};
};

template<typename Tsrc>
class RcTracker final {
public:
    const Tsrc& source() const;
    Tsrc& source();
    RcState query();

    static constexpr uint8_t recovery_rate = Tsrc::recovery_rate;

private:
    RcCommand cache_;
    Ticker<uint8_t> tolerance_;
    Tsrc source_;
};

template<typename... Tsrcs>
class RcMux final {
public:
    template<std::size_t N>
    using ElementType = typename std::tuple_element<N, std::tuple < Tsrcs...> >::type;

    template<std::size_t N>
    inline ElementType<N>& source() {
        return std::get<N>(sources_).source();
    }

    RcState query();
    void setFilter(uint8_t filter);

private:
    template<std::size_t I = 0>
    inline typename std::enable_if<I == sizeof...(Tsrcs), RcState>::type queryHelper() {
        invalid_count_ += 1;
        return RcState{RcState::State::Timeout, {}};
    }

    template<std::size_t I = 0>
    inline typename std::enable_if<(I < sizeof...(Tsrcs)), RcState>::type queryHelper() {
        RcState state = std::get<I>(sources_).query();
        if (state.state != RcState::State::Timeout && filter_.accepts(1 << I)) {
            invalid_count_ -= std::get<I>(sources_).recovery_rate;
            return state;
        }
        return queryHelper<I + 1>();
    }

    RcFilter filter_;
    std::tuple<RcTracker<Tsrcs>...> sources_;
    bool valid_{true};
    int16_t invalid_count_{0};
};

#include "RcMux_impl.h"

#endif // RC_MUX_H
