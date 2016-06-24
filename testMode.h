/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <testMode.h/cpp>

    Test mode for LED and motor functionality

*/

class LED;
class Motors;
class State;

void runTestMode(State& state, LED& led, Motors& motors);
