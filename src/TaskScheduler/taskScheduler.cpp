#include "api.h"
#include "TaskScheduler/Task/taskInterface.h"
#include "taskScheduler.h"

void TaskScheduler::checkForTasks(float t) {
    
    for (auto task_i = taskList.begin(); task_i != taskList.end(); ) {
        if ((*task_i)->GetExecutionTime() <= t) {
            (*task_i)->performTask();
            task_i = taskList.erase(task_i); // Erase returns the next iterator
        } else {
            ++task_i; // Increment the iterator only if no erase occurs
        }
    }
};