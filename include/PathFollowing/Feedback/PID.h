#pragma once

#ifndef PID_H
#define PID_H

class PID {
    private:
        float kP;
        float kI;
        float kD;
        float maxInt = 10;
        float intBound = 3;
        float error = 1000;
        float prevError = 0;
        float integral = 0;
        float derivative = 0;
        float target;
        float settle_error = 3;
    public:
        PID(float kP, float kI, float kD);
        void UpdateTarget(float target, float settle_error);
        void UpdateIntegralValues(float intBound, float maxInt);
        float getError() const { return error; }

        float UpdateValues(float current_pos);
};

#endif //PID_H