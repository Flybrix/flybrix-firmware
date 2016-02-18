/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "airframe.h"

#include "config.h"  //CONFIG variable

#include "state.h"

Airframe::Airframe(State* __state) {
    state = __state;
}

uint16_t Airframe::mix(int32_t mFz, int32_t mTx, int32_t mTy, int32_t mTz) {
    int32_t mmax = max(max(mFz, mTx), max(mTy, mTz));
    return (uint16_t) constrain( (mFz * state->Fz + mTx * state->Tx + mTy * state->Ty + mTz * state->Tz) / mmax, 0, 4095);
}

void Airframe::updateMotorsMix() {
    state->MotorOut[0] = mix(CONFIG.data.mixTableFz[0], CONFIG.data.mixTableTx[0], CONFIG.data.mixTableTy[0], CONFIG.data.mixTableTz[0]); 
    state->MotorOut[1] = mix(CONFIG.data.mixTableFz[1], CONFIG.data.mixTableTx[1], CONFIG.data.mixTableTy[1], CONFIG.data.mixTableTz[1]); 
    state->MotorOut[2] = mix(CONFIG.data.mixTableFz[2], CONFIG.data.mixTableTx[2], CONFIG.data.mixTableTy[2], CONFIG.data.mixTableTz[2]); 
    state->MotorOut[3] = mix(CONFIG.data.mixTableFz[3], CONFIG.data.mixTableTx[3], CONFIG.data.mixTableTy[3], CONFIG.data.mixTableTz[3]); 
    state->MotorOut[4] = mix(CONFIG.data.mixTableFz[4], CONFIG.data.mixTableTx[4], CONFIG.data.mixTableTy[4], CONFIG.data.mixTableTz[4]); 
    state->MotorOut[5] = mix(CONFIG.data.mixTableFz[5], CONFIG.data.mixTableTx[5], CONFIG.data.mixTableTy[5], CONFIG.data.mixTableTz[5]); 
    state->MotorOut[6] = mix(CONFIG.data.mixTableFz[6], CONFIG.data.mixTableTx[6], CONFIG.data.mixTableTy[6], CONFIG.data.mixTableTz[6]); 
    state->MotorOut[7] = mix(CONFIG.data.mixTableFz[7], CONFIG.data.mixTableTx[7], CONFIG.data.mixTableTy[7], CONFIG.data.mixTableTz[7]); 
}
