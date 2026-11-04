/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "airframe.h"
#include <Arduino.h>
#include "controlVectors.h"
#include "debug.h"

namespace {
// Scales `desired` by one factor in [0, 1] so that no channel rises above its remaining headroom (`available`)
// or falls below zero (`allocated`). Scaling the whole request uniformly keeps the direction of the correction,
// and checks every channel against its own limits (the old code only checked the single largest request and
// divided by mix table entries, which could be zero).
void scaleToFit(int32_t* desired, const int32_t* available, const int32_t* allocated) {
    // track the scale as the fraction room / need, compared with exact integer cross-multiplication
    int64_t best_room = 1;
    int64_t best_need = 1;
    for (size_t i = 0; i < 8; ++i) {
        int64_t need{0};
        int64_t room{0};
        if (desired[i] > 0) {
            need = desired[i];
            room = max(available[i], (int32_t)0);
        } else if (desired[i] < 0) {
            need = -(int64_t)desired[i];
            room = max(allocated[i], (int32_t)0);
        } else {
            continue;
        }
        if (need > room && room * best_need < best_room * need) {
            best_room = room;
            best_need = need;
        }
    }
    if (best_room == best_need) {
        return;  // everything fits
    }
    for (size_t i = 0; i < 8; ++i) {
        desired[i] = (int32_t)(((int64_t)desired[i] * best_room) / best_need);
    }
}

void allocate(int32_t* desired, int32_t* remaining, int32_t* allocated) {
    scaleToFit(desired, remaining, allocated);
    for (size_t i = 0; i < 8; ++i) {
        allocated[i] += desired[i];
        remaining[i] -= desired[i];
    }
}
}  // namespace

uint32_t motor_update_count{0};
uint32_t motor_clipping[8]{0};
int32_t min_level[8]{0};
int32_t max_level[8]{0};

void Airframe::setMotorsToMixTable(const ControlVectors& controls) {
    const int32_t tx_reserve = 200;
    const int32_t ty_reserve = 200;
    const int32_t tz_reserve = 200;
    int32_t remaining_power[8];
    int32_t allocated_power[8];
    int32_t fz_power[8];
    int32_t txy_power[8];
    int32_t tz_power[8];
    for (size_t i = 0; i < 8; ++i) {
        int32_t mmax = max(max(abs(mix_table.fz[i]), abs(mix_table.tx[i])), max(abs(mix_table.ty[i]), abs(mix_table.tz[i])));
        if (mmax == 0) {  // unused channel (e.g. a quad on the 8 channel board)
            fz_power[i] = txy_power[i] = tz_power[i] = 0;
        } else {
            fz_power[i] = mix_table.fz[i] * controls.force_z / mmax;
            // pitch and roll are allocated together so that neither axis loses authority first when motors saturate
            txy_power[i] = mix_table.tx[i] * controls.torque_x / mmax + mix_table.ty[i] * controls.torque_y / mmax;
            tz_power[i] = mix_table.tz[i] * controls.torque_z / mmax;
        }
        remaining_power[i] = 4095 - tx_reserve - ty_reserve - tz_reserve;
        allocated_power[i] = 0;
    }

    // allocate thrust
    for (size_t i = 0; i < 8; ++i) {
        allocated_power[i] += min( remaining_power[i], fz_power[i] );
        remaining_power[i] -= allocated_power[i];

        fz_power[i] -= allocated_power[i]; // keep track of residual thrust for later use
    }

    // allocate pitch and roll
    for (size_t i = 0; i < 8; ++i) {
        remaining_power[i] += tx_reserve + ty_reserve;
    }
    allocate(txy_power, remaining_power, allocated_power);

    // allocate yaw
    for (size_t i = 0; i < 8; ++i) {
        remaining_power[i] += tz_reserve;
    }
    allocate(tz_power, remaining_power, allocated_power);

    // if we have room, allocate the residual thrust
    allocate(fz_power, remaining_power, allocated_power);

    for (size_t i = 0; i < 8; ++i) {
        if (allocated_power[i] == 0) {
            continue;
        }
        else {
            motor_update_count++;
            break;
        }
    }

    for (size_t i = 0; i < 8; ++i) {
        if (allocated_power[i] < 0) {
            if (allocated_power[i] < min_level[i]) {
                min_level[i] = allocated_power[i];
            }
            allocated_power[i] = 0;
            motor_clipping[i]++;
        }
        if (allocated_power[i] > 4095) {
            if (allocated_power[i] > max_level[i]) {
                max_level[i] = allocated_power[i];
            }
            allocated_power[i] = 4095;
            motor_clipping[i]++;
        }
        if (motor_clipping[i] && (motor_clipping[i] % 1500 == 0)){
            DebugPrintf("Motor channel %d is saturating (%2.0f%%) [%6d,%6d]", i, (float) 100.0f * motor_clipping[i] / motor_update_count, min_level[i], max_level[i]);
        }

        motors_.set(i, (uint16_t) allocated_power[i]);
    }
}

void Airframe::setMotor(size_t index, uint16_t value) {
    motors_.set(index, value);
}

void Airframe::resetMotors() {
    motors_.reset();
}

void Airframe::enableMotors() {
    enabled_ = true;
}

void Airframe::disableMotors() {
    enabled_ = false;
}

void Airframe::setOverride(bool override) {
    override_ = override;
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
