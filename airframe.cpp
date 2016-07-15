/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "airframe.h"
#include <Arduino.h>
#include "state.h"

Airframe::Airframe(State* state) : state(state) {
}

uint16_t Airframe::mix(int32_t mFz, int32_t mTx, int32_t mTy, int32_t mTz) {
    int32_t mmax = max(max(mFz, mTx), max(mTy, mTz));
    return constrain((mFz * state->Fz + mTx * state->Tx + mTy * state->Ty + mTz * state->Tz) / mmax, 0, 4095);
}

void Airframe::updateMotorsMix() {
    for (size_t i = 0; i < 8; ++i)
        state->MotorOut[i] = mix(mix_table.fz[i], mix_table.tx[i], mix_table.ty[i], mix_table.tz[i]);
}
