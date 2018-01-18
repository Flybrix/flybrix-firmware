/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "airframe.h"
#include <Arduino.h>
#include "controlVectors.h"

void constrainPower(int32_t *desired_power, int32_t *available_power, int32_t *allocated_power, int8_t* mix_table_values){
    int32_t max_desired_power = 0;
    int32_t min_desired_power = 0;
    size_t max_i = 0;
    size_t min_i = 0;
    for (size_t i = 0; i < 8; ++i) {
        if (desired_power[i] > max_desired_power ) {
            max_desired_power = desired_power[i];
            max_i = i;
        }
        if (desired_power[i] < min_desired_power ) {
            min_desired_power = desired_power[i];
            min_i = i;
        }
    }
    
    int32_t additive_power_deficit = 0;    //we want to add more power than remains available
    int32_t subtractive_power_deficit = 0; //we want to subtract more power than we have allocated thus far
    
    if ( (max_desired_power > 0) && (max_desired_power > available_power[max_i])) {
        additive_power_deficit = max_desired_power - available_power[max_i];
    }
    if ( (min_desired_power < 0) && (-min_desired_power > allocated_power[min_i])) {
        subtractive_power_deficit = -min_desired_power - allocated_power[min_i];
    }
     
    if ( additive_power_deficit > subtractive_power_deficit) {
        int32_t scaled_deficit = additive_power_deficit / ((int32_t) mix_table_values[max_i]);
        for (size_t i = 0; i < 8; ++i) {
            //desired_power[i] -= scaled_deficit * (int32_t) mix_table_values[i];
        }   
    }
    else {
        int32_t scaled_deficit = subtractive_power_deficit / ((int32_t) mix_table_values[min_i]);
        for (size_t i = 0; i < 8; ++i) {
            //desired_power[i] += scaled_deficit * (int32_t) mix_table_values[i];
        }  
    }
    
}

void Airframe::setMotorsToMixTable(const ControlVectors& controls) {
    const int32_t tx_reserve = 200;
    const int32_t ty_reserve = 200;
    const int32_t tz_reserve = 200;
    int32_t remaining_power[8];
    int32_t allocated_power[8];
    int32_t fz_power[8];
    int32_t tx_power[8];
    int32_t ty_power[8];
    int32_t tz_power[8];
    for (size_t i = 0; i < 8; ++i) {
        int32_t mmax = max(max(abs(mix_table.fz[i]), abs(mix_table.tx[i])), max(abs(mix_table.ty[i]), abs(mix_table.tz[i])));
        fz_power[i] = mix_table.fz[i] * controls.force_z  / mmax;
        tx_power[i] = mix_table.tx[i] * controls.torque_x / mmax;
        ty_power[i] = mix_table.ty[i] * controls.torque_y / mmax;
        tz_power[i] = mix_table.tz[i] * controls.torque_z / mmax;
        remaining_power[i] = 4095 - tx_reserve - ty_reserve - tz_reserve;
        allocated_power[i] = 0;
    }
    
    // allocate thrust
    for (size_t i = 0; i < 8; ++i) {
        allocated_power[i] += min( remaining_power[i], fz_power[i] );
        remaining_power[i] -= allocated_power[i];
        
        fz_power[i] -= allocated_power[i]; // keep track of residual thrust for later use
    }
    
    // allocate roll
    for (size_t i = 0; i < 8; ++i) {
        remaining_power[i] += ty_reserve;
    }
    constrainPower(ty_power, remaining_power, allocated_power, mix_table.ty);
    for (size_t i = 0; i < 8; ++i) {
        allocated_power[i] += ty_power[i];
        remaining_power[i] -= allocated_power[i];
    }

    // allocate pitch
    for (size_t i = 0; i < 8; ++i) {
        remaining_power[i] += tx_reserve;
    }
    constrainPower(tx_power, remaining_power, allocated_power, mix_table.tx);
    for (size_t i = 0; i < 8; ++i) {
        allocated_power[i] += tx_power[i];
        remaining_power[i] -= allocated_power[i];
    }

    // allocate yaw
    for (size_t i = 0; i < 8; ++i) {
        remaining_power[i] += tz_reserve;
    }
    constrainPower(tz_power, remaining_power, allocated_power, mix_table.tz);
    for (size_t i = 0; i < 8; ++i) {
        allocated_power[i] += tz_power[i];
        remaining_power[i] -= allocated_power[i];
    }   

    // if we have room, allocate the residual thrust
    constrainPower(fz_power, remaining_power, allocated_power, mix_table.fz);
    for (size_t i = 0; i < 8; ++i) {
        allocated_power[i] += fz_power[i];
        remaining_power[i] -= allocated_power[i];
    }    

    // set the levels
    for (size_t i = 0; i < 8; ++i) {
        motors_.set(i, (uint16_t) constrain(allocated_power[i], 0, 4095));
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
