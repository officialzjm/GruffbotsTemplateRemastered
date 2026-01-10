#pragma once

#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

class Feedforward {
    private:
        float Ks;
        float Kv;
        float Ka; 
    public:
        Feedforward(float Ks, float Kv, float Ka);
        float GetTargetVoltage(float target_veloc, float target_accel);
};

#endif //FEEDFORWARD_H