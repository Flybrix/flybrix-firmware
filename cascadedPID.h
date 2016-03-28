/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
    *

    <cascadedPID.h>

*/

#ifndef CASCADED_PID_h
#define CASCADED_PID_h

#include "PID.h"

class CascadedPID final {
   public:
    CascadedPID(float* master_terms, float* slave_terms);

    void isMasterWrapped(bool wrapped = true) {
        master_.isWrapped(wrapped);
    }

    void isSlaveWrapped(bool wrapped = true) {
        slave_.isWrapped(wrapped);
    }

    const PID& master() const {
        return master_;
    }

    const PID& slave() const {
        return slave_;
    }

    void setSetpoint(float v) {
        setpoint_ = v;
    }

    float getScalingFactor(bool use_master, bool use_slave, float default_val) {
        if (use_master)
            return master_.commandToValue();
        if (use_slave)
            return slave_.commandToValue();
        return default_val;
    }

    void setMasterInput(float v) {
        master_.setInput(v);
    }

    void setSlaveInput(float v) {
        slave_.setInput(v);
    }

    float Compute(uint32_t now, bool use_master, bool use_slave);

    void IntegralReset();

   private:
    PID master_, slave_;
    float setpoint_{0.0f};
};

#endif
