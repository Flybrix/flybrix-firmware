/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "RcCommand.h"

RcCommand::AuxMask RcCommand::auxMask() const {
    return RcCommand::AuxMask(aux1_) | (RcCommand::AuxMask(aux2_) << 3);
}

RcCommand::AUX RcCommand::aux1() const {
    return aux1_;
}

RcCommand::AUX RcCommand::aux2() const {
    return aux2_;
}

RcCommand::Axis RcCommand::throttle() const {
    return throttle_;
}

RcCommand::Axis RcCommand::pitch() const {
    return pitch_;
}

RcCommand::Axis RcCommand::roll() const {
    return roll_;
}

RcCommand::Axis RcCommand::yaw() const {
    return yaw_;
}

void RcCommand::resetAxes() {
    setThrottle(0);
    setPitch(0);
    setRoll(0);
    setYaw(0);
}

void RcCommand::setThrottle(RcCommand::Axis value) {
    throttle_ = value;
}

void RcCommand::setPitch(RcCommand::Axis value) {
    pitch_ = value;
}

void RcCommand::setRoll(RcCommand::Axis value) {
    roll_ = value;
}

void RcCommand::setYaw(RcCommand::Axis value) {
    yaw_ = value;
}

void RcCommand::parseBools(bool l1, bool m1, bool h1, bool l2, bool m2, bool h2) {
    aux1_ = RcCommand::parseSingleBools(l1, m1, h1);
    aux2_ = RcCommand::parseSingleBools(l2, m2, h2);
}

void RcCommand::parseAuxMask(RcCommand::AuxMask auxmask) {
    parseBools(auxmask & 1, auxmask & 2, auxmask & 4, auxmask & 8, auxmask & 16, auxmask & 32);
}

RcCommand::AUX RcCommand::parseSingleBools(bool low, bool mid, bool high) {
    if (low) {
        return AUX::Low;
    }
    if (mid) {
        return AUX::Mid;
    }
    if (high) {
        return AUX::High;
    }
    return AUX::None;
}
