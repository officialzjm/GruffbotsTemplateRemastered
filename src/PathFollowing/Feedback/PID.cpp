#include "PID.h"

PID::PID(float kP, float kI, float kD):
kP(kP), kI(kI), kD(kD)
{
}

void PID::UpdateTarget(float target, float settle_error)
{
    this->target = target;
    this->settle_error = settle_error;
}

void PID::UpdateIntegralValues(float intBound, float maxInt)
{
    this->intBound = intBound;
    this->maxInt = maxInt;
}

float PID::UpdateValues(float current_pos)
{
    error = target-current_pos;
    integral += error;
    if(error <= intBound) integral = 0;
    if(integral > maxInt) integral = maxInt;
    derivative = error - prevError;
    prevError = error;

    return (kP*error)+(kI*integral)+(kD*derivative);
}
