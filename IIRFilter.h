/*
    *  Flybrix Flight Controller -- Copyright 2019 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef IIR_FILTER_H
#define IIR_FILTER_H

class IIRFilter {
public:
    IIRFilter(float output, float time_constant);
    float update(float in, float dt);

private:
    float out_;
    float tau_;
};

#endif // IIR_FILTER_H
