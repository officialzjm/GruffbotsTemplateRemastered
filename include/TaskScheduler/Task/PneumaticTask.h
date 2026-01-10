#pragma once

#ifndef PNEUMATICTASK_H
#define PNEUMATICTASK_H

#include "api.h"
#include "TaskScheduler/Task/taskInterface.h"
#include "TaskScheduler/taskScheduler.h"

class PneumaticTask : public TaskInterface {
    public:
    void performTask() override {
        piston->set_value(true);
    }
    PneumaticTask(pros::ADIDigitalOut* piston, float t) {
        this->piston = piston;
        this->execution_time = t;
    }
    private:
    pros::ADIDigitalOut* piston;
};

#endif //PNEUMATICTASK_H