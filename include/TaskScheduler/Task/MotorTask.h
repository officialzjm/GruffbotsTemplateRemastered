#pragma once

#ifndef MOTORTASK_H
#define MOTORTASK_H

#include "api.h"
#include "taskInterface.h"

class MotorTask : public TaskInterface {
    public:
    void performTask() override {
        motor->move_voltage(voltage);
    }
    MotorTask(pros::Motor* motor, float voltage, float t) {
        this->motor = motor;
        this->voltage = voltage;
        this->execution_time = t;
    }
    private:
        pros::Motor* motor;
        float voltage;
};

#endif //MOTORTASK_H