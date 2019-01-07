/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "PPMChannel.h"
#include "Debug.h"

bool PPMchannel::validateDeadzoneAndMidpoint(uint16_t deadzone, uint16_t midpoint, bool log) {
    bool validMidpoint = PPMchannel::isValidMidpoint(midpoint);
    bool validDeadzone = PPMchannel::isValidDeadzoneForMidpoint(deadzone, midpoint);

    if (log) {
        if (!validMidpoint) {
            DebugPrint("Channel midpoints must be within the channel range");
        }
        if (!validDeadzone) {
            DebugPrint("Channel deadzone cannot be larger than the midpoint value and range");
        }
    }

    return validMidpoint && validDeadzone;
}
