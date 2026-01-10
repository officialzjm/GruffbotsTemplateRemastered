#pragma once

#ifndef TASKSCHEDULER_H
#define TASKSCHEDULER_H

#include "TaskScheduler/Task/taskInterface.h"
#include "api.h"

class TaskScheduler {

private:
    std::vector<TaskInterface*> taskList;
public:
    void addTask(TaskInterface* task) {
        taskList.push_back(task);
    };
    void checkForTasks(float t);
};

#endif //TASKSCHEDULER_H