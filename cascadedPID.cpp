/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
    *

    <cascadedPID.h>

*/

#include "cascadedPID.h"

CascadedPID::CascadedPID(const float* master_terms, const float* slave_terms) : master_(master_terms), slave_(slave_terms) {
}

float CascadedPID::Compute(uint32_t now, bool use_master, bool use_slave) {
    float value{setpoint_};
    if (use_master) {
        master_.setSetpoint(value);
        value = master_.Compute(now);
    } else {
        master_.setTimer(now);
    }
    if (use_slave) {
        slave_.setSetpoint(value);
        value = slave_.Compute(now);
    } else {
        slave_.setTimer(now);
    }
    return value;
};

void CascadedPID::IntegralReset() {
    master_.IntegralReset();
    slave_.IntegralReset();
}
