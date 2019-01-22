/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef RCSTATE_H
#define RCSTATE_H

#include "RcCommand.h"

struct RcState final {
    enum class State {
        Ok,
        Waiting,
        Timeout,
    };

    State state;
    RcCommand command;
};

#endif // RCSTATE_H
