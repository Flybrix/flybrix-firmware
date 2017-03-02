/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "airframe.h"
#include <Arduino.h>
#include "state.h"
#include "stateFlag.h"
#include "controlVectors.h"

Airframe::Airframe(StateFlag& flag) : flag_(flag) {
}

uint16_t mix(const ControlVectors& controls, int32_t mFz, int32_t mTx, int32_t mTy, int32_t mTz) {
    int32_t mmax = max(max(mFz, mTx), max(mTy, mTz));
    return constrain((mFz * controls.force_z + mTx * controls.torque_x + mTy * controls.torque_y + mTz * controls.torque_z) / mmax, 0, 4095);
}

void Airframe::setMotorsToMixTable(const ControlVectors& controls) {
    for (size_t i = 0; i < 8; ++i)
        motors_.set(i, mix(controls, mix_table.fz[i], mix_table.tx[i], mix_table.ty[i], mix_table.tz[i]));
}

void Airframe::setMotor(size_t index, uint16_t value) {
    motors_.set(index, value);
}

void Airframe::resetMotors() {
    motors_.reset();
}

void Airframe::enableMotors() {
    enabled_ = true;
    flag_.set(Status::ENABLED);
}

void Airframe::disableMotors() {
    enabled_ = false;
    flag_.clear(Status::ENABLED);
}

void Airframe::setOverride(bool override) {
    override_ = override;
    if (override_) {
        flag_.set(Status::OVERRIDE);
    } else {
        flag_.clear(Status::OVERRIDE);
    }
}

void Airframe::applyChanges(const ControlVectors& control) {
    if (!override_) {
        setMotorsToMixTable(control);
    }
    motors_.updateAllChannels(enabled_ || override_);
}

bool Airframe::motorsEnabled() const {
    return enabled_;
}

bool Airframe::motorsOverridden() const {
    return override_;
}

// default configuration is the flat8 octocopter:
//  * CH0 ( CW: red +, blue -, type A prop) at full front right
//  * CH2 (CCW: wht +, blk  -, type B prop) at mid front right
//  * CH4 ( CW: red +, blue -, type A prop) at mid rear right
//  * CH6 (CCW: wht +, blk  -, type B prop) at full rear right
//  * CH1 (CCW: wht +, blk  -, type B prop) at full front left
//  * CH3 ( CW: red +, blue -, type A prop) at mid front left
//  * CH5 (CCW: wht +, blk  -, type B prop) at mid rear left
//  * CH7 ( CW: red +, blue -, type A prop) at rull rear left
// Note that the same mixtable can be used to build a quad on
// CH0, CH6, CH1, CH7
//
// pitch positive (nose up) needs a Tx negative restoring torque -->
// (Tx<0) should drop the nose by increasing rear channels and decreasing
// front channels
// roll positive (right side down) needs a Ty negative restoring torque -->
// (Ty<0) should raise the right side by increasing right channels and
// decreasing left channels
// yaw positive (CCW rotation from top down) needs a Tz negative restoring
// torque --> (Tz<0) should decrease CCW motors & increase CW motors

Airframe::MixTable::MixTable() : fz{1, 1, 1, 1, 1, 1, 1, 1}, tx{1, 1, 1, 1, -1, -1, -1, -1}, ty{-1, 1, -1, 1, -1, 1, -1, 1}, tz{1, -1, -1, 1, 1, -1, -1, 1} {
}
