/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef KINEMATICS_H
#define KINEMATICS_H

using KinematicsFloat = float;

struct Rotation {
    KinematicsFloat pitch{0.0};  // x
    KinematicsFloat roll{0.0};   // y
    KinematicsFloat yaw{0.0};    // z
};

static_assert(sizeof(Rotation) == 12, "Rotation data is not packed");

struct Kinematics final {
    // radians
    Rotation angle;
    // radians/sec
    Rotation rate;

    KinematicsFloat altitude{0.0};  // meters
};

#endif /* KINEMATICS_H */
