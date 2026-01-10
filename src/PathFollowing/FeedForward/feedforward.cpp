#include "feedforward.h"
#include "utils.h"

Feedforward::Feedforward(float Ks, float Kv, float Ka)
{
    this->Ks = Ks;
    this->Kv = Kv;
    this->Ka = Ka;
}

float Feedforward::GetTargetVoltage(float target_veloc, float target_accel)
{
    float Voltage = Ks*utils::sign(target_veloc) + Kv*target_veloc + Ka*target_accel;
    return Voltage;
}
