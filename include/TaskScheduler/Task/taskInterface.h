#pragma once

#ifndef TASKINTERFACE_H
#define TASKINTERFACE_H

#include "api.h"

class TaskInterface {
    public:
        virtual void performTask()=0;
        float GetExecutionTime() {
            return execution_time;
        }
    protected:
        float execution_time;
};

#endif //TASKINTERFACE_H