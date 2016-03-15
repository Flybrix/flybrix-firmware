/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <airframe.h/cpp>

    Airframe translates the four control vectors (thrust force, pitch torque, roll torque, yaw torque) into Motor Levels
*/

#ifndef airframe_h
#define airframe_h

#include <cstdint>

class State;

class Airframe {
   public:
    Airframe(State* state);
    void updateMotorsMix();

   private:
    uint16_t mix(int32_t mFz, int32_t mTx, int32_t mTy, int32_t mTz);
    State* state;
};

#endif
